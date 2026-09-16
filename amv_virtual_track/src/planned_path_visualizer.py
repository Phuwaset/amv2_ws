#!/usr/bin/env python3
import rospy
import csv
import os
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray


class PlannedPathVisualizer:

    def __init__(self):
        rospy.init_node('planned_path_visualizer', anonymous=False)

        self.station_csv = rospy.get_param('~station_csv', os.path.expanduser('~/amv_ws/src/amv_service/station/station_list_office.csv'))
        self.waypoint_csv = rospy.get_param('~waypoint_csv', os.path.expanduser('~/amv_ws/src/amv_service/station/waypoint_follower_list.csv'))
        self.max_connect_dist = rospy.get_param('~max_connect_dist', 1.8)
        self.frame_id = rospy.get_param('~frame_id', 'map')

        self.nodes = {}
        self.build_graph()
        self.print_diagnostics()

        self.marker_pub = rospy.Publisher('/planned_path_markers', MarkerArray, queue_size=1, latch=True)
        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_cb)

        self.publish_graph_markers()
        rospy.loginfo('[PlannedPathVisualizer] Online. Trigger: rostopic pub -1 /amv/command/goto_station std_msgs/String "data: \'Line1\'"')
        rospy.spin()

    def build_graph(self):
        self.nodes = {}
        if os.path.exists(self.station_csv):
            with open(self.station_csv, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    name = (row.get('name') or '').strip()
                    if name:
                        self.nodes[name] = {
                            'name': name, 'x': float(row['x']), 'y': float(row['y']),
                            'qz': float(row.get('qz', 0.0)), 'qw': float(row.get('qw', 1.0)),
                            'type': 'station'}
        if os.path.exists(self.waypoint_csv):
            with open(self.waypoint_csv, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    name = (row.get('name') or '').strip()
                    if name:
                        self.nodes[name] = {
                            'name': name, 'x': float(row['x']), 'y': float(row['y']),
                            'qz': float(row.get('qz', 0.0)), 'qw': float(row.get('qw', 1.0)),
                            'type': 'waypoint'}
        self.adj = {}
        for n1_name, n1 in self.nodes.items():
            self.adj[n1_name] = []
            for n2_name, n2 in self.nodes.items():
                if n1_name == n2_name:
                    continue
                d = math.hypot(n1['x'] - n2['x'], n1['y'] - n2['y'])
                if d <= self.max_connect_dist:
                    self.adj[n1_name].append((n2_name, round(d, 2)))

    def print_diagnostics(self):
        rospy.loginfo('[Graph] max_connect_dist = %.2f m', self.max_connect_dist)
        rospy.loginfo('[Graph] nodes = %d (stations + waypoints)', len(self.nodes))
        edges = 0
        isolated = []
        for name, nbrs in self.adj.items():
            edges += len(nbrs)
            if not nbrs:
                isolated.append(name)
        rospy.loginfo('[Graph] undirected edges = %d', edges // 2)
        if isolated:
            rospy.logwarn('[Graph] ISOLATED nodes (cannot route through): %s', ', '.join(isolated))
        else:
            rospy.loginfo('[Graph] All nodes connected - route possible.')
        for name, nbrs in self.adj.items():
            rospy.loginfo('[Graph]   %s (%s) -> %s', name, self.nodes[name]['type'], nbrs)

    def publish_graph_markers(self):
        arr = MarkerArray()
        for i, (name, n) in enumerate(self.nodes.items()):
            sphere = Marker()
            sphere.header.frame_id = self.frame_id
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = 'check_nodes'
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = n['x']
            sphere.pose.position.y = n['y']
            sphere.pose.position.z = 0.05
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.15
            if n['type'] == 'station':
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 1.0, 0.0, 0.0, 0.95
            else:
                sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = 0.0, 0.6, 1.0, 0.95
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = rospy.Time.now()
            text.ns = 'check_labels'
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = n['x']
            text.pose.position.y = n['y']
            text.pose.position.z = 0.35
            text.scale.z = 0.15
            text.color.r = text.color.g = text.color.b = text.color.a = 1.0
            text.text = name
            arr.markers.append(text)

        edge_marker = Marker()
        edge_marker.header.frame_id = self.frame_id
        edge_marker.header.stamp = rospy.Time.now()
        edge_marker.ns = 'check_edges'
        edge_marker.id = 1000
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.scale.x = 0.02
        edge_marker.color.r, edge_marker.color.g, edge_marker.color.b, edge_marker.color.a = 0.6, 0.6, 0.6, 0.8
        for name, nbrs in self.adj.items():
            for nb, _ in nbrs:
                if name < nb:
                    p1 = Point()
                    p1.x, p1.y, p1.z = self.nodes[name]['x'], self.nodes[name]['y'], 0.02
                    p2 = Point()
                    p2.x, p2.y, p2.z = self.nodes[nb]['x'], self.nodes[nb]['y'], 0.02
                    edge_marker.points.append(p1)
                    edge_marker.points.append(p2)
        arr.markers.append(edge_marker)
        self.marker_pub.publish(arr)

    def path_cb(self, msg):
        arr = MarkerArray()

        line = Marker()
        line.header.frame_id = msg.header.frame_id
        line.header.stamp = rospy.Time.now()
        line.ns = 'planned_path_line'
        line.id = 0
        line.type = Marker.LINE_STRIP
        line.action = Marker.ADD
        line.scale.x = 0.06
        line.color.r, line.color.g, line.color.b, line.color.a = 0.0, 0.5, 1.0, 1.0
        for pose in msg.poses:
            p = Point()
            p.x, p.y, p.z = pose.pose.position.x, pose.pose.position.y, pose.pose.position.z + 0.1
            line.points.append(p)
        arr.markers.append(line)

        step = max(1, len(msg.poses) // 20)
        for i in range(0, len(msg.poses), step):
            pose = msg.poses[i]
            arrow = Marker()
            arrow.header.frame_id = msg.header.frame_id
            arrow.header.stamp = rospy.Time.now()
            arrow.ns = 'planned_path_arrows'
            arrow.id = i
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose.position.x = pose.pose.position.x
            arrow.pose.position.y = pose.pose.position.y
            arrow.pose.position.z = pose.pose.position.z + 0.1
            arrow.pose.orientation = pose.pose.orientation
            arrow.scale.x = 0.18
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.45, 0.0, 0.95
            arr.markers.append(arrow)

        rospy.loginfo('[PlannedPathVisualizer] /planned_path received: %d poses (frame %s). Drew line + arrows.', len(msg.poses), msg.header.frame_id)
        self.marker_pub.publish(arr)


if __name__ == '__main__':
    try:
        PlannedPathVisualizer()
    except rospy.ROSInterruptException:
        pass
