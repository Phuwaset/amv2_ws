#!/usr/bin/env python3
import rospy
import csv
import math
import os
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Trigger, TriggerResponse


def normalize_quaternion(qx, qy, qz, qw):
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm < 1e-6:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


class StationPlottingTest():

    CSV_HEADER = ['name', 'mode', 'timer', 'pin', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw', 'theta']

    def __init__(self):
        rospy.init_node('station_plotting_test', anonymous=False)

        self.csv_path = rospy.get_param('~station_path', default='/home/minirw/amv_ws/src/amv_service/station/station_list_office.csv')
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self.stations = []
        self.load_stations()

        self.web_pub = rospy.Publisher('station_markers_p2p', MarkerArray, queue_size=1, latch=True)
        self.rviz_pub = rospy.Publisher('station_labels', MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.initialpose_callback)

        self.srv_undo = rospy.Service('/amv/recorder/undo_station', Trigger, self.srv_undo_cb)
        self.srv_clear = rospy.Service('/amv/recorder/clear_station', Trigger, self.srv_clear_cb)
        self.srv_save = rospy.Service('/amv/recorder/save_station', Trigger, self.srv_save_cb)

        self.publish_all()
        rospy.loginfo('[StationPlottingTest] Online. Drag "2D Pose Estimate" on RViz to pin a station.')
        rospy.spin()

    def srv_undo_cb(self, req):
        if self.stations:
            popped = self.stations.pop()
            self.save_csv()
            self.publish_all()
            return TriggerResponse(success=True, message='Removed station [{}].'.format(popped['name']))
        return TriggerResponse(success=False, message='Station list is empty.')

    def srv_clear_cb(self, req):
        self.stations = []
        self.save_csv()
        self.publish_all()
        return TriggerResponse(success=True, message='Cleared all stations.')

    def srv_save_cb(self, req):
        self.save_csv()
        return TriggerResponse(success=True, message='Stations saved successfully.')

    def load_stations(self):
        self.stations = []
        if not os.path.exists(self.csv_path):
            rospy.logwarn('[StationPlottingTest] CSV not found: %s (will create on save)', self.csv_path)
            return
        with open(self.csv_path, mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                name = (row.get('name') or '').strip()
                if not name:
                    continue
                qx, qy, qz, qw = normalize_quaternion(
                    float(row.get('qx', 0.0)),
                    float(row.get('qy', 0.0)),
                    float(row.get('qz', 0.0)),
                    float(row.get('qw', 1.0)))
                self.stations.append({
                    'name': name,
                    'mode': (row.get('mode') or 'Auto').strip(),
                    'timer': (row.get('timer') or '0').strip(),
                    'pin': (row.get('pin') or 'Up').strip(),
                    'x': float(row.get('x', 0.0)),
                    'y': float(row.get('y', 0.0)),
                    'z': float(row.get('z', 0.0)),
                    'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
                    'theta': (row.get('theta') or '0').strip(),
                })
        rospy.loginfo('[StationPlottingTest] Loaded %d station(s) from %s', len(self.stations), self.csv_path)

    def save_csv(self):
        os.makedirs(os.path.dirname(self.csv_path), exist_ok=True)
        with open(self.csv_path, mode='w', encoding='utf-8', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=self.CSV_HEADER)
            writer.writeheader()
            for s in self.stations:
                writer.writerow(s)
        rospy.loginfo('[StationPlottingTest] Saved %d station(s) to %s', len(self.stations), self.csv_path)

    def initialpose_callback(self, data):
        p = data.pose.pose
        qx, qy, qz, qw = normalize_quaternion(
            p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w)
        rospy.loginfo('[StationPlottingTest] Pose captured (%.3f, %.3f) -> answer prompts in terminal', p.position.x, p.position.y)
        try:
            name = input('Station name: ').strip()
            if not name:
                rospy.logwarn('[StationPlottingTest] Empty name -> station skipped')
                return
            mode = input('Mode (Auto/Manual): ').strip() or 'Auto'
            timer = input('Timer (sec): ').strip() or '1'
            pin = input('Pin (Up/Down): ').strip() or 'Up'
        except EOFError:
            rospy.logwarn('[StationPlottingTest] No terminal input available -> station skipped (run this node in a terminal to pin)')
            return

        self.stations.append({
            'name': name,
            'mode': mode,
            'timer': timer,
            'pin': pin,
            'x': p.position.x,
            'y': p.position.y,
            'z': p.position.z,
            'qx': qx, 'qy': qy, 'qz': qz, 'qw': qw,
            'theta': '0',
        })
        self.save_csv()
        self.publish_all()
        rospy.loginfo('[StationPlottingTest] Station "%s" pinned at (%.3f, %.3f)', name, p.position.x, p.position.y)

    def publish_all(self):
        self.web_pub.publish(self.build_web_markers())
        self.rviz_pub.publish(self.build_rviz_markers())

    def build_web_markers(self):
        arr = MarkerArray()
        for i, s in enumerate(self.stations):
            name_marker = Marker()
            name_marker.header.frame_id = self.frame_id
            name_marker.header.stamp = rospy.Time.now()
            name_marker.ns = 'stations_name'
            name_marker.id = i
            name_marker.type = Marker.TEXT_VIEW_FACING
            name_marker.action = Marker.ADD
            name_marker.lifetime = rospy.Duration(0)
            name_marker.scale.z = 0.5
            name_marker.color.r, name_marker.color.g, name_marker.color.b, name_marker.color.a = 0.0, 0.0, 1.0, 1.0
            name_marker.pose.position.x = s['x']
            name_marker.pose.position.y = s['y']
            name_marker.pose.position.z = s['z']
            name_marker.text = s['name']
            arr.markers.append(name_marker)

        line_marker = Marker()
        line_marker.header.frame_id = self.frame_id
        line_marker.header.stamp = rospy.Time.now()
        line_marker.ns = 'stations_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_LIST
        line_marker.action = Marker.ADD
        line_marker.lifetime = rospy.Duration(0)
        line_marker.scale.x = 0.04
        line_marker.scale.y = 0.04
        line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a = 0.0, 0.0, 1.0, 1.0
        line_marker.points = list()
        for i in range(len(self.stations) - 1):
            p1 = line_marker.points[-1] if line_marker.points else None
            from geometry_msgs.msg import Point
            pt1 = Point()
            pt1.x, pt1.y, pt1.z = self.stations[i]['x'], self.stations[i]['y'], self.stations[i]['z']
            pt2 = Point()
            pt2.x, pt2.y, pt2.z = self.stations[i + 1]['x'], self.stations[i + 1]['y'], self.stations[i + 1]['z']
            line_marker.points.append(pt1)
            line_marker.points.append(pt2)
        if len(self.stations) > 2:
            from geometry_msgs.msg import Point
            pt1 = Point()
            pt1.x, pt1.y, pt1.z = self.stations[-1]['x'], self.stations[-1]['y'], self.stations[-1]['z']
            pt2 = Point()
            pt2.x, pt2.y, pt2.z = self.stations[0]['x'], self.stations[0]['y'], self.stations[0]['z']
            line_marker.points.append(pt1)
            line_marker.points.append(pt2)

        circle_marker = Marker()
        circle_marker.header.frame_id = self.frame_id
        circle_marker.header.stamp = rospy.Time.now()
        circle_marker.ns = 'stations_circle'
        circle_marker.id = 0
        circle_marker.type = Marker.SPHERE_LIST
        circle_marker.action = Marker.ADD
        circle_marker.lifetime = rospy.Duration(0)
        circle_marker.scale.x = 0.5
        circle_marker.scale.y = 0.5
        circle_marker.scale.z = 0.5
        circle_marker.color.r, circle_marker.color.g, circle_marker.color.b, circle_marker.color.a = 1.0, 0.0, 0.0, 1.0
        circle_marker.points = list()
        for s in self.stations:
            from geometry_msgs.msg import Point
            pt = Point()
            pt.x, pt.y, pt.z = s['x'], s['y'], s['z']
            circle_marker.points.append(pt)

        arr.markers.append(line_marker)
        arr.markers.append(circle_marker)
        return arr

    def build_rviz_markers(self):
        arr = MarkerArray()
        for i, s in enumerate(self.stations):
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = rospy.Time.now()
            text_marker.ns = 'station_names'
            text_marker.id = i
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.lifetime = rospy.Duration(0)
            text_marker.pose.position.x = s['x']
            text_marker.pose.position.y = s['y']
            text_marker.pose.position.z = 0.28
            text_marker.scale.z = 0.14
            text_marker.color.r, text_marker.color.g, text_marker.color.b, text_marker.color.a = 1.0, 0.85, 0.0, 1.0
            text_marker.text = '[{}] ({}s)'.format(s['name'], s['timer'])
            arr.markers.append(text_marker)

            arrow_marker = Marker()
            arrow_marker.header.frame_id = self.frame_id
            arrow_marker.header.stamp = rospy.Time.now()
            arrow_marker.ns = 'station_arrows'
            arrow_marker.id = i + 100
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.lifetime = rospy.Duration(0)
            arrow_marker.pose.position.x = s['x']
            arrow_marker.pose.position.y = s['y']
            arrow_marker.pose.position.z = 0.05
            arrow_marker.pose.orientation.z = s['qz']
            arrow_marker.pose.orientation.w = s['qw']
            arrow_marker.scale.x = 0.35
            arrow_marker.scale.y = 0.06
            arrow_marker.scale.z = 0.06
            arrow_marker.color.r, arrow_marker.color.g, arrow_marker.color.b, arrow_marker.color.a = 1.0, 0.45, 0.0, 0.95
            arr.markers.append(arrow_marker)
        return arr


if __name__ == '__main__':
    StationPlottingTest()
