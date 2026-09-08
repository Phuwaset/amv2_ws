#!/usr/bin/env python3
import rospy
import csv
import math
import os
from visualization_msgs.msg import Marker, MarkerArray

def normalize_quaternion(qz, qw):
    norm = math.hypot(qz, qw)
    if norm < 1e-6:
        return 0.0, 1.0
    return qz / norm, qw / norm

class StationMarkerNode:
    def __init__(self):
        rospy.init_node('station_marker_node', anonymous=False)

        self.csv_path = rospy.get_param('~csv_path', os.path.expanduser('~/turtlebot3_ws/src/amv_virtual_track/config/station_list.csv'))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        
        self.marker_pub = rospy.Publisher('/station_labels', MarkerArray, queue_size=1, latch=True)
        self.publish_station_markers()

    def publish_station_markers(self):
        if not os.path.exists(self.csv_path):
            rospy.logwarn(f"[StationMarker] CSV not found at: {self.csv_path}")
            return

        marker_arr = MarkerArray()
        with open(self.csv_path, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for i, row in enumerate(reader):
                name = row.get('name', '').strip()
                if not name:
                    continue

                x = float(row['x'])
                y = float(row['y'])
                raw_qz = float(row.get('qz', 0.0))
                raw_qw = float(row.get('qw', 1.0))
                timer = row.get('timer', '0')
                qz, qw = normalize_quaternion(raw_qz, raw_qw)

                # 1. ป้ายชื่อสถานีสีเหลืองทอง
                text_marker = Marker()
                text_marker.header.frame_id = self.frame_id
                text_marker.header.stamp = rospy.Time.now()
                text_marker.ns = "station_names"
                text_marker.id = i
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = x
                text_marker.pose.position.y = y
                text_marker.pose.position.z = 0.28
                text_marker.scale.z = 0.14
                text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 0.85, 0.0, 1.0
                text_marker.text = f"[{name}] ({timer}s)"
                marker_arr.markers.append(text_marker)

                # 2. ลูกศรสถานีสีส้มสด
                arrow_marker = Marker()
                arrow_marker.header.frame_id = self.frame_id
                arrow_marker.header.stamp = rospy.Time.now()
                arrow_marker.ns = "station_arrows"
                arrow_marker.id = i + 100
                arrow_marker.type = Marker.ARROW
                arrow_marker.action = Marker.ADD
                arrow_marker.pose.position.x = x
                arrow_marker.pose.position.y = y
                arrow_marker.pose.position.z = 0.05
                arrow_marker.pose.orientation.z = qz
                arrow_marker.pose.orientation.w = qw
                arrow_marker.scale.x = 0.35
                arrow_marker.scale.y = 0.06
                arrow_marker.scale.z = 0.06
                arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = 1.0, 0.45, 0.0, 0.95
                marker_arr.markers.append(arrow_marker)

        self.marker_pub.publish(marker_arr)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = StationMarkerNode()
        node.run()
    except rospy.ROSInterruptException:
        pass