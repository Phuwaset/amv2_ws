#!/usr/bin/env python3
import rospy
import csv
import math
import os
import heapq
import tf2_ros

from std_msgs.msg import String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class BranchingPathPlanner:
    def __init__(self):
        rospy.init_node('path_planner_node', anonymous=False)

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.station_csv = rospy.get_param('~station_csv', os.path.expanduser('~/amv_ws/src/amv_service/station/station_list_office.csv'))
        self.waypoint_csv = rospy.get_param('~waypoint_csv', os.path.expanduser('~/amv_ws/src/amv_service/station/waypoint_follower_list.csv'))
        self.track_csv = rospy.get_param('~track_csv', os.path.expanduser('~/amv_ws/src/amv_service/station/manual_track.csv'))
        
        # สวิตช์บังคับสองทาง: True = บังคับให้ทุกเส้นวิ่งไป-กลับได้, False = อิงตามคอลัมน์ mode ใน manual_track.csv
        self.force_twoway = rospy.get_param('~force_twoway', False)
        
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')
        self.step_size = rospy.get_param('~step_size', 0.05)
        self.max_connect_dist = rospy.get_param('~max_connect_dist', 3.0)

        # ---------------------------------------------------------
        # TF & ROS Interfaces
        # ---------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=1, latch=True)
        self.cmd_sub = rospy.Subscriber('/amv/command/goto_station', String, self.command_cb)

        self.nodes = {}       # โหนดทั้งหมด {node_name: {x, y, qz, qw, type}}
        self.adj_list = {}    # โครงข่ายเส้นเชื่อม {node_name: [(neighbor_name, dist), ...]}
        self.stations = {}    # รายชื่อสถานีหลักสำหรับค้นหา {station_lower: node_data}

        self.load_graph_data()
        rospy.loginfo("[BranchingPathPlanner] Graph Routing System Online.")

    def load_graph_data(self):
        self.nodes = {}
        self.stations = {}
        self.adj_list = {}

        # 1. โหลดข้อมูลสถานีหลัก
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

        # 2. โหลดข้อมูล Waypoints
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

        # กำหนด Adjacent List ว่างสำหรับทุกโหนด
        for n_name in self.nodes.keys():
            self.adj_list[n_name] = []

        # แมปชื่อโหนดแบบ case-insensitive เพื่อป้องกันปัญหาพิมพ์ตัวพิมพ์เล็ก-ใหญ่ต่างกันใน CSV
        lower_to_node = {k.lower(): k for k in self.nodes.keys()}

        # 3. สร้าง Graph เชื่อมต่อ
        if os.path.exists(self.track_csv):
            rospy.loginfo(f"[Planner] Loading explicit tracks from: {self.track_csv}")
            loaded_edges = 0

            with open(self.track_csv, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    from_raw = row.get('from_node', '').strip()
                    to_raw = row.get('to_node', '').strip()
                    mode = row.get('mode', 'bidirectional').strip().lower()

                    u = lower_to_node.get(from_raw.lower())
                    v = lower_to_node.get(to_raw.lower())

                    if u and v:
                        d = math.hypot(self.nodes[u]['x'] - self.nodes[v]['x'],
                                       self.nodes[u]['y'] - self.nodes[v]['y'])
                        
                        # สร้างเส้น u -> v
                        self.adj_list[u].append((v, d))
                        loaded_edges += 1

                        # ถ้าเป็น bidirectional หรือเปิด force_twoway ให้สร้างเส้น v -> u
                        if mode == 'bidirectional' or self.force_twoway:
                            self.adj_list[v].append((u, d))
                            loaded_edges += 1
                    else:
                        missing = []
                        if not u: missing.append(from_raw)
                        if not v: missing.append(to_raw)
                        rospy.logwarn(f"[Planner] Edge skipped: {missing} not defined in CSVs.")

            rospy.loginfo(f"[Planner] Loaded {loaded_edges} directed tracks successfully.")
        else:
            # Fallback หากยังไม่ได้สร้าง manual_track.csv ให้ต่อตามระยะ
            rospy.logwarn(f"[Planner] '{self.track_csv}' not found! Falling back to proximity graph.")
            for n1_name, n1 in self.nodes.items():
                for n2_name, n2 in self.nodes.items():
                    if n1_name == n2_name:
                        continue
                    d = math.hypot(n1['x'] - n2['x'], n1['y'] - n2['y'])
                    if d <= self.max_connect_dist:
                        self.adj_list[n1_name].append((n2_name, d))

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rospy.Time(0), rospy.Duration(0.1))
            return trans.transform.translation.x, trans.transform.translation.y
        except Exception:
            return None, None

    def find_shortest_path(self, start_node, goal_node):
        """Dijkstra Algorithm ค้นหาเส้นทางที่ผ่านเฉพาะ Edge ที่กำหนดไว้"""
        queue = [(0.0, start_node, [start_node])]
        visited = set()

        while queue:
            cost, curr, path = heapq.heappop(queue)

            if curr == goal_node:
                return path

            if curr in visited:
                continue
            visited.add(curr)

            for neighbor, weight in self.adj_list.get(curr, []):
                if neighbor not in visited:
                    heapq.heappush(queue, (cost + weight, neighbor, path + [neighbor]))
        return None

    def command_cb(self, msg):
        target_name = msg.data.strip()
        
        # รีโหลดข้อมูลทุกครั้งที่รับคำสั่ง (สามารถแก้ manual_track.csv แล้วสั่งวิ่งได้ทันทีโดยไม่ต้องรีสตาร์ทโหนด)
        self.load_graph_data()

        if target_name.lower() not in self.stations:
            rospy.logerr(f"[Planner] Station '{target_name}' not found in database!")
            return

        target_key = self.stations[target_name.lower()]['name']
        rx, ry = self.get_robot_pose()
        if rx is None:
            rospy.logerr(f"[Planner] Cannot acquire TF transform from '{self.frame_id}' to '{self.base_frame}'!")
            return

        # 1. หาโหนดที่มีทางเชื่อม (degree > 0) และใกล้ตัวรถที่สุด
        connectable_nodes = [k for k, v in self.adj_list.items() if len(v) > 0]
        if not connectable_nodes:
            rospy.logerr("[Planner] No connected nodes found in graph!")
            return

        nearest_node = min(connectable_nodes, key=lambda k: math.hypot(self.nodes[k]['x'] - rx, self.nodes[k]['y'] - ry))

        # 2. คำนวณเส้นทาง
        route = self.find_shortest_path(nearest_node, target_key)
        if not route:
            rospy.logerr(f"[Planner] No valid route from '{nearest_node}' to '{target_key}'! Check manual_track.csv connections.")
            return

        rospy.loginfo(f"[Planner] Active Route: {' -> '.join(route)}")

        # 3. Interpolate จุดเส้นทางออกเป็น nav_msgs/Path
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

        # จุดสุดท้ายส่ง Orientation ตามค่าของสถานีเป้าหมาย
        if path_msg.poses:
            target_node_data = self.nodes[target_key]
            path_msg.poses[-1].pose.orientation.z = target_node_data['qz']
            path_msg.poses[-1].pose.orientation.w = target_node_data['qw']

        self.path_pub.publish(path_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        planner = BranchingPathPlanner()
        planner.run()
    except rospy.ROSInterruptException:
        pass