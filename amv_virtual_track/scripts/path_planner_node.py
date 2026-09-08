#!/usr/bin/env python3
import rospy
import csv
import math
import os
import tf2_ros
from collections import deque

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class BranchingPathPlanner:
    def __init__(self):
        rospy.init_node('path_planner_node', anonymous=False)

        # โหลดพารามิเตอร์ไฟล์
        self.station_csv = rospy.get_param('~station_csv', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/station_list.csv'))
        self.waypoint_csv = rospy.get_param('~waypoint_csv', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/waypoint_follower_list.csv'))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.step_size = rospy.get_param('~step_size', 0.05)
        self.max_connect_dist = rospy.get_param('~max_connect_dist', 1.8) # ระยะเชื่อมโยงโหนดทางแยก

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.cmd_sub = rospy.Subscriber('/amv/command/goto_station', String, self.command_cb)

        self.nodes = {}       # รวมทั้ง Waypoints และ Stations เข้าเป็น Node กราฟ
        self.adj_list = {}    # รายการเชื่อมต่อเส้นทาง (Graph Edges)
        self.stations = {}
        
        self.load_graph_data()
        rospy.loginfo("[BranchingPathPlanner] Graph Routing System Online.")

    def load_graph_data(self):
        self.nodes = {}
        self.stations = {}
        self.adj_list = {}

        # 1. โหลดสถานีหลัก
        if os.path.exists(self.station_csv):
            with open(self.station_csv, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    name = row.get('name', '').strip()
                    if name:
                        node_data = {
                            'name': name,
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'qz': float(row.get('qz', 0.0)),
                            'qw': float(row.get('qw', 1.0)),
                            'type': 'station'
                        }
                        self.nodes[name] = node_data
                        self.stations[name.lower()] = node_data

        # 2. โหลด Waypoints ทางแยก/ทางเดิน
        if os.path.exists(self.waypoint_csv):
            with open(self.waypoint_csv, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    name = row.get('name', '').strip()
                    if name:
                        self.nodes[name] = {
                            'name': name,
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'qz': float(row.get('qz', 0.0)),
                            'qw': float(row.get('qw', 1.0)),
                            'type': 'waypoint'
                        }

        # 3. สร้าง Graph เชื่อมต่อโหนดทางแยกตามระยะทาง (Proximity Graph Builder)
        for n1_name, n1 in self.nodes.items():
            self.adj_list[n1_name] = []
            for n2_name, n2 in self.nodes.items():
                if n1_name == n2_name:
                    continue
                d = math.hypot(n1['x'] - n2['x'], n1['y'] - n2['y'])
                if d <= self.max_connect_dist:
                    self.adj_list[n1_name].append((n2_name, d))

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, 'base_footprint', rospy.Time(0), rospy.Duration(0.1))
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception:
            return None, None

    def find_shortest_path(self, start_node, goal_node):
        """Dijkstra Search เพื่อหาเส้นทางผ่านจุดตัดทางแยกที่ถูกต้อง"""
        queue = [(0, start_node, [start_node])]
        visited = set()

        while queue:
            queue.sort(key=lambda x: x[0])
            cost, curr, path = queue.pop(0)

            if curr == goal_node:
                return path

            if curr in visited:
                continue
            visited.add(curr)

            for neighbor, weight in self.adj_list.get(curr, []):
                if neighbor not in visited:
                    queue.append((cost + weight, neighbor, path + [neighbor]))
        return None

    def command_cb(self, msg):
        target_name = msg.data.strip()
        self.load_graph_data()

        if target_name.lower() not in self.stations:
            rospy.logerr(f"[Planner] Station '{target_name}' not found!")
            return

        target_key = self.stations[target_name.lower()]['name']
        rx, ry = self.get_robot_pose()
        if rx is None:
            rospy.logerr("[Planner] Cannot acquire AMCL robot pose!")
            return

        # 1. หาโหนด/ทางแยกที่ใกล้ตัวรถที่สุด
        nearest_node = min(self.nodes.keys(), key=lambda k: math.hypot(self.nodes[k]['x'] - rx, self.nodes[k]['y'] - ry))

        # 2. คำนวณเส้นทางผ่านโครงข่ายทางแยก
        route = self.find_shortest_path(nearest_node, target_key)
        if not route:
            rospy.logerr(f"[Planner] No valid route to '{target_key}'!")
            return

        rospy.loginfo(f"[Planner] Active Route: {' -> '.join(route)}")

        # 3. สร้าง Sub-waypoints ออกมาเป็น nav_msgs/Path
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp = rospy.Time.now()

        route_pts = [{'x': rx, 'y': ry}] + [self.nodes[n] for n in route]
        for i in range(len(route_pts) - 1):
            p1, p2 = route_pts[i], route_pts[i+1]
            dx, dy = p2['x'] - p1['x'], p2['y'] - p1['y']
            dist = math.hypot(dx, dy)
            yaw = math.atan2(dy, dx)
            steps = max(1, int(dist / self.step_size))

            for s in range(steps):
                r = s / float(steps)
                pose = PoseStamped()
                pose.header.frame_id = self.frame_id
                pose.pose.position.x = p1['x'] + (dx * r)
                pose.pose.position.y = p1['y'] + (dy * r)
                pose.pose.orientation.z = math.sin(yaw / 2.0)
                pose.pose.orientation.w = math.cos(yaw / 2.0)
                path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = BranchingPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass