#!/usr/bin/env python3
import rospy
import os
import csv
import yaml
import math
import tf.transformations

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse

def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm

class WaypointRecorderNode:
    def __init__(self):
        rospy.init_node('waypoint_recorder_node', anonymous=False)

        # 1. พารามิเตอร์ไฟล์และเฟรม
        self.csv_path = rospy.get_param('~csv_path', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/waypoint_follower_list.csv'))
        self.yaml_path = rospy.get_param('~yaml_path', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/waypoints.yaml'))
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self.waypoints = []
        self.load_existing_waypoints()

        # 2. Publishers & Subscribers
        self.marker_pub = rospy.Publisher('/waypoint_markers', MarkerArray, queue_size=1, latch=True)
        self.goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.goal_callback)

        # 3. Services สำหรับจัดการจุด
        self.srv_undo = rospy.Service('/amv/recorder/undo', Trigger, self.srv_undo_cb)
        self.srv_clear = rospy.Service('/amv/recorder/clear', Trigger, self.srv_clear_cb)
        self.srv_save = rospy.Service('/amv/recorder/save', Trigger, self.srv_save_cb)

        self.publish_markers()
        rospy.loginfo(f"[WaypointRecorder] Online on frame [{self.frame_id}]. Click '2D Nav Goal' on RViz to record waypoints.")

    def load_existing_waypoints(self):
        self.waypoints = []
        if os.path.exists(self.csv_path):
            try:
                with open(self.csv_path, mode='r', encoding='utf-8') as f:
                    reader = csv.DictReader(f)
                    for row in reader:
                        name = row.get('name', '').strip()
                        if name:
                            qx, qy, qz, qw = normalize_quaternion(
                                float(row.get('qx', 0.0)),
                                float(row.get('qy', 0.0)),
                                float(row.get('qz', 0.0)),
                                float(row.get('qw', 1.0))
                            )
                            self.waypoints.append({
                                'name': name,
                                'x': float(row['x']),
                                'y': float(row['y']),
                                'z': float(row.get('z', 0.0)),
                                'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw
                            })
                rospy.loginfo(f"[WaypointRecorder] Loaded {len(self.waypoints)} waypoints from CSV.")
            except Exception as e:
                rospy.logwarn(f"[WaypointRecorder] Cannot read CSV: {e}")

    def save_to_disk(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=['name', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
            writer.writeheader()
            for wp in self.waypoints:
                writer.writerow(wp)

        os.makedirs(os.path.dirname(self.yaml_path), exist_ok=True)
        yaml_data = {'waypoints': self.waypoints}
        with open(self.yaml_path, mode='w', encoding='utf-8') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)

    def goal_callback(self, msg):
        qx, qy, qz, qw = normalize_quaternion(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        )

        wp_idx = len(self.waypoints) + 1
        wp_name = f"wy{wp_idx}"

        new_wp = {
            'name': wp_name,
            'x': round(msg.pose.position.x, 4),
            'y': round(msg.pose.position.y, 4),
            'z': 0.0,
            'qx': round(qx, 4),
            'qy': round(qy, 4),
            'qz': round(qz, 4),
            'qw': round(qw, 4)
        }

        self.waypoints.append(new_wp)
        self.save_to_disk()
        self.publish_markers()
        rospy.loginfo(f"[WaypointRecorder] Recorded [{wp_name}] at ({new_wp['x']}, {new_wp['y']})")

    def publish_markers(self):
        marker_array = MarkerArray()
        
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, wp in enumerate(self.waypoints):
            # จุดกลมสีฟ้า
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "waypoint_spheres"
            sphere.id = i * 3
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = wp['x']
            sphere.pose.position.y = wp['y']
            sphere.pose.position.z = wp['z'] + 0.05
            sphere.scale.x = 0.12
            sphere.scale.y = 0.12
            sphere.scale.z = 0.12
            sphere.color.r = 0.0
            sphere.color.g = 0.6
            sphere.color.b = 1.0
            sphere.color.a = 0.9
            marker_array.markers.append(sphere)

            # ป้ายชื่อข้อความ
            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = "waypoint_labels"
            text.id = i * 3 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = wp['x']
            text.pose.position.y = wp['y']
            text.pose.position.z = wp['z'] + 0.25
            text.scale.z = 0.12
            text.color.r = 1.0
            text.color.g = 1.0
            text.color.b = 1.0
            text.color.a = 1.0
            text.text = wp['name']
            marker_array.markers.append(text)

            # ลูกศรชี้ทิศทาง
            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = "waypoint_arrows"
            arrow.id = i * 3 + 2
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = wp['x']
            arrow.pose.position.y = wp['y']
            arrow.pose.position.z = wp['z'] + 0.05
            arrow.pose.orientation.x = wp['qx']
            arrow.pose.orientation.y = wp['qy']
            arrow.pose.orientation.z = wp['qz']
            arrow.pose.orientation.w = wp['qw']
            arrow.scale.x = 0.25
            arrow.scale.y = 0.04
            arrow.scale.z = 0.04
            arrow.color.r = 1.0
            arrow.color.g = 0.3
            arrow.color.b = 0.0
            arrow.color.a = 0.8
            marker_array.markers.append(arrow)

        self.marker_pub.publish(marker_array)

    def srv_undo_cb(self, req):
        if self.waypoints:
            popped = self.waypoints.pop()
            self.save_to_disk()
            self.publish_markers()
            return TriggerResponse(success=True, message=f"Removed [{popped['name']}].")
        return TriggerResponse(success=False, message="Waypoint list is empty.")

    def srv_clear_cb(self, req):
        self.waypoints = []
        self.save_to_disk()
        self.publish_markers()
        return TriggerResponse(success=True, message="Cleared all waypoints.")

    def srv_save_cb(self, req):
        self.save_to_disk()
        return TriggerResponse(success=True, message="Waypoints saved successfully.")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        recorder = WaypointRecorderNode()
        recorder.run()
    except rospy.ROSInterruptException:
        pass
