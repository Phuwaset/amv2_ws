#!/usr/bin/env python3
import rospy
import tf2_ros
import tf.transformations
import math
import os
import csv

from geometry_msgs.msg import Twist, Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger, TriggerResponse

def normalize_quaternion(qz, qw):
    norm = math.hypot(qz, qw)
    if norm < 1e-6:
        return 0.0, 1.0
    return qz / norm, qw / norm

class RegulatedWaypointController:
    def __init__(self):
        rospy.init_node('waypoint_controller_node', anonymous=False)

        # -------------------------------------------------------------
        # 1. Parameters (Regulated Pure Pursuit)
        # -------------------------------------------------------------
        self.csv_path = rospy.get_param('~csv_path', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/station_list.csv'))
        self.waypoint_csv = rospy.get_param('~waypoint_csv', os.path.expanduser('~/amv_ws/src/amv_virtual_track/config/waypoint_follower_list.csv'))
        self.frame_id = rospy.get_param('~frame_id', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_footprint')

        # Limiters
        self.lookahead_dist = rospy.get_param('~lookahead_dist', 0.50)
        self.max_lin_vel = rospy.get_param('~max_lin_vel', 0.60)
        self.min_lin_vel = rospy.get_param('~min_lin_vel', 0.08) # ความเร็วคลานขั้นต่ำ
        self.max_ang_vel = rospy.get_param('~max_ang_vel', 0.35)
        
        # Ramp & Approach Limits
        self.max_accel = rospy.get_param('~max_accel', 0.25)
        self.max_decel = rospy.get_param('~max_decel', 0.35)
        self.decel_dist_station = rospy.get_param('~decel_dist_station', 0.30) # ระยะเริ่มชะลอ
        
        # Tolerances
        self.station_dist_tol = rospy.get_param('~station_dist_tol', 0.15) # ระยะตัดเข้าจอด
        self.yaw_tol = rospy.get_param('~yaw_tol', 0.04)
        self.startup_check_tol = rospy.get_param('~startup_check_tol', 0.50)

        # Startup
        self.depart_v = rospy.get_param('~depart_v', 0.20) # ความเร็วช่วงออกตัวตรงจากสถานี (m/s)
        self.depart_dist = rospy.get_param('~depart_dist', 0.50) # เดินตรงครบเท่านี้แล้วค่อยเข้า Pure Pursuit (m)
        self.station_depart_tol = rospy.get_param('~station_depart_tol', 0.60) # รัศมีที่ถือว่า "อยู่ที่สถานี" ถึงจะเข้า DEPART

        # Dock Entry Checkpoint (จุดเช็คก่อนเข้าสถานี: คำนวณจาก x/y/yaw ใน CSV)
        self.entry_check_dist = rospy.get_param('~entry_check_dist', 0.50) # ระยะจุดเช็คล่วงหน้า (m)
        self.entry_trigger_dist = rospy.get_param('~entry_trigger_dist', 1.20) # เริ่มสลับเป้าไปจุดเช็คเมื่อใกล้สถานี (m)
        self.entry_hold_dist = rospy.get_param('~entry_hold_dist', 0.15) # รัศมีนับว่าถึงจุดเช็ค (m)
        self.entry_approach_v = rospy.get_param('~entry_approach_v', 0.15) # ความเร็วช่วงลัดเข้าจุดเช็ค (m/s)
        self.entry_linear_v = rospy.get_param('~entry_linear_v', 0.10) # ความเร็วเส้นคงที่: หันหน้าถูกแล้ว (m/s)
        self.dock_align_tol = rospy.get_param('~dock_align_tol', 0.07) # ระยะหยุดเทียบจากศูนย์กลางสถานี
        self.goal_confirm_loops = rospy.get_param('~goal_confirm_loops', 6) # กันจอดหลอกตอน TF/AMCL เด้ง (20Hz -> ~0.3s)
        self.goal_confirm_count = 0

        # -------------------------------------------------------------
        # 2. Internal State
        # -------------------------------------------------------------
        self.stations = {}
        self.waypoints = []
        self.load_database()

        self.path_poses = []
        self.is_active = True
        self.state = 'IDLE'
        self.last_path_idx = 0
        self.depart_dist_traveled = 0.0
        
        self.current_v = 0.0
        self.last_time = rospy.Time.now()
        self.wait_start_time = None
        self.target_station_data = None

        # -------------------------------------------------------------
        # 3. ROS Interfaces
        # -------------------------------------------------------------
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.cte_pub = rospy.Publisher('/amv/diagnostics/cross_track_error', Float32, queue_size=1)
        self.status_pub = rospy.Publisher('/amv/diagnostics/status', String, queue_size=1)
        self.local_plan_pub = rospy.Publisher('/amv/local_plan', Path, queue_size=1)

        self.path_sub = rospy.Subscriber('/planned_path', Path, self.path_callback)

        self.srv_start = rospy.Service('/start_mission', Trigger, self.srv_start_cb)
        self.srv_stop = rospy.Service('/stop_mission', Trigger, self.srv_stop_cb)
        self.srv_resume = rospy.Service('/resume_manual', Trigger, self.srv_resume_cb)

        self.rate = rospy.Rate(20)

    def load_database(self):
        self.stations = {}
        if os.path.exists(self.csv_path):
            with open(self.csv_path, mode='r', encoding='utf-8') as f:
                for row in csv.DictReader(f):
                    name = row.get('name', '').strip()
                    if name:
                        qz, qw = normalize_quaternion(float(row.get('qz', 0.0)), float(row.get('qw', 1.0)))
                        self.stations[name.lower()] = {
                            'name': name,
                            'mode': row.get('mode', 'Auto').strip(),
                            'timer': float(row.get('timer', 0.0)),
                            'x': float(row['x']),
                            'y': float(row['y']),
                            'qz': qz,
                            'qw': qw
                        }

    def get_robot_pose(self):
        try:
            trans = self.tf_buffer.lookup_transform(self.frame_id, self.base_frame, rospy.Time(0), rospy.Duration(0.05))
            x = trans.transform.translation.x
            y = trans.transform.translation.y
            rot = trans.transform.rotation
            (_, _, yaw) = tf.transformations.euler_from_quaternion([rot.x, rot.y, rot.z, rot.w])
            return x, y, yaw
        except Exception:
            return None, None, None

    def startup_pose_verification(self):
        rospy.loginfo("[Init] Waiting 2.0s for AMCL & TF localization to settle...")
        rospy.sleep(2.0)
        self.load_database()

        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            rx, ry, _ = self.get_robot_pose()
            if rx is None:
                rospy.logwarn_throttle(2.0, f"[Init] Waiting for TF '{self.frame_id} -> {self.base_frame}'...")
                rate.sleep()
                continue
            self.state = 'IDLE'
            break

    def path_callback(self, msg):
        if not msg.poses:
            return
        self.path_poses = msg.poses
        self.load_database()
        self.last_path_idx = 0  

        last_p = self.path_poses[-1].pose.position
        min_d = float('inf')
        self.target_station_data = None
        for st in self.stations.values():
            d = math.hypot(st['x'] - last_p.x, st['y'] - last_p.y)
            if d < min_d and d <= 0.35:
                min_d = d
                self.target_station_data = st

        self.state = 'FOLLOWING'
        st_name = self.target_station_data['name'] if self.target_station_data else "Target"

        # ถ้าอยู่นิ่งและอยู่ที่สถานีจริง เริ่มด้วยเฟส DEPART (ยิงตรงตามหัวปัจจุบัน ไม่ใช้ PP จนครบระยะ)
        cur_x, cur_y, cur_yaw = self.get_robot_pose()
        if cur_yaw is not None and self.current_v < 0.05 and self.is_near_station(cur_x, cur_y):
            self.depart_dist_traveled = 0.0
            self.state = 'DEPART'
            rospy.loginfo(f"[Controller] Path loaded -> Target: [{st_name}]. Departing straight {self.depart_dist} m @ {self.depart_v} m/s, then Pure Pursuit.")
        else:
            rospy.loginfo(f"[Controller] Path loaded -> Target: [{st_name}]. Executing Regulated Pure Pursuit.")

    def is_near_station(self, cur_x, cur_y):
        for st in self.stations.values():
            if math.hypot(st['x'] - cur_x, st['y'] - cur_y) <= self.station_depart_tol:
                return True
        return False

    def station_yaw(self, st):
        (_, _, yaw) = tf.transformations.euler_from_quaternion([0.0, 0.0, st['qz'], st['qw']])
        return yaw

    def direct_rotate(self, yaw_err):
        # หมุนอยู่กับที่แบบ Hysteresis (บน Error ใหญ่ -> เร็ว, ใกล้จบ -> หน่วง) กันหัวสั่น
        if abs(yaw_err) > 0.10:
            return max(min(1.0 * yaw_err, 0.25), -0.25)
        return max(min(0.5 * yaw_err, 0.12), -0.12)

    def srv_start_cb(self, req):
        self.is_active = True
        return TriggerResponse(success=True, message="Mission Resumed/Started.")

    def srv_stop_cb(self, req):
        self.is_active = False
        self.current_v = 0.0
        self.cmd_pub.publish(Twist())
        return TriggerResponse(success=True, message="Mission Paused.")

    def srv_resume_cb(self, req):
        if self.state == 'MANUAL_WAIT':
            self.state = 'IDLE'
            return TriggerResponse(success=True, message="Manual wait released.")
        return TriggerResponse(success=False, message="Not in MANUAL_WAIT state.")

    def get_regulated_lookahead(self, cur_x, cur_y, dist_to_goal):
        """ค้นหาจุดเป้าหมาย โดยใช้กฎ RPP: ป้องกันจุดเป้าหมายกระโดดไปด้านหลังตอนใกล้จอด"""
        if not self.path_poses or len(self.path_poses) < 2:
            return None, 0.0

        # 1. หาจุดใกล้สุดแบบ Forward-Only
        search_start = self.last_path_idx
        search_end = min(len(self.path_poses), search_start + 40)
        
        min_cte = float('inf')
        best_idx = search_start
        for i in range(search_start, search_end):
            pose = self.path_poses[i]
            d = math.hypot(pose.pose.position.x - cur_x, pose.pose.position.y - cur_y)
            if d < min_cte:
                min_cte = d
                best_idx = i
                
        self.last_path_idx = best_idx

        # 2. RPP Goal Approach Heuristic: ถ้าใกล้ถึงเป้าหมาย ให้ล็อกเป้าไปที่จุดปลายทางเลย
        goal_pt = self.path_poses[-1].pose.position
        if dist_to_goal <= self.lookahead_dist:
            return goal_pt, min_cte

        # 3. สะสมระยะทางหาจุด Lookahead ตามปกติ
        accum_dist = 0.0
        target_idx = best_idx
        for i in range(best_idx, len(self.path_poses) - 1):
            p1 = self.path_poses[i].pose.position
            p2 = self.path_poses[i + 1].pose.position
            accum_dist += math.hypot(p2.x - p1.x, p2.y - p1.y)
            if accum_dist >= self.lookahead_dist:
                target_idx = i + 1
                break

        return self.path_poses[target_idx].pose.position, min_cte

    def run(self):
        self.startup_pose_verification()

        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = max(0.01, min(0.1, (now - self.last_time).to_sec()))
            self.last_time = now

            if not self.is_active:
                self.rate.sleep()
                continue

            try:
                cur_x, cur_y, cur_yaw = self.get_robot_pose()
                if cur_x is None:
                    self.rate.sleep()
                    continue
            except Exception as e:
                rospy.logerr_throttle(2.0, f"[Controller] TF error: {e}")
                self.rate.sleep()
                continue

            cmd = Twist()

            # -------------------------------------------------------------
            # STATE 0: DEPART (ออกตัวจากสถานีแบบตรง ไม่คำนวณ PP จนกว่าจะเกินระยะ)
            # -------------------------------------------------------------
            if self.state == 'DEPART':
                self.status_pub.publish(String(data="DEPART_STRAIGHT"))

                # เดินหน้า 0 ความเร็ว depart_v แบบ ramp นุ่ม
                if self.depart_v > self.current_v:
                    self.current_v = min(self.depart_v, self.current_v + self.max_accel * dt)
                else:
                    self.current_v = max(self.depart_v, self.current_v - self.max_decel * dt)

                # ตั้งตรงเป๊ะ: ล็อกหัวไว้ตามเดิม ไม่หมุน ไม่คำนวณอะไร (w = 0)
                cmd.linear.x = self.current_v
                cmd.angular.z = 0.0

                self.depart_dist_traveled += self.current_v * dt
                if self.depart_dist_traveled >= self.depart_dist:
                    rospy.loginfo(f"[Depart] Straight {self.depart_dist_traveled:.2f} m done. Entering Pure Pursuit.")
                    self.state = 'FOLLOWING'

            # -------------------------------------------------------------
            # STATE 1: IDLE 
            # -------------------------------------------------------------
            elif self.state == 'IDLE':
                self.status_pub.publish(String(data="STANDBY_IDLE"))
                self.current_v = 0.0
                cmd.linear.x, cmd.angular.z = 0.0, 0.0

            # -------------------------------------------------------------
            # STATE 2: FOLLOWING (Regulated Pure Pursuit)
            # -------------------------------------------------------------
            elif self.state == 'FOLLOWING':
                self.status_pub.publish(String(data="AGV_RUN"))
                
                goal_p = self.path_poses[-1].pose.position
                dist_to_goal = math.hypot(goal_p.x - cur_x, goal_p.y - cur_y)
                
                target_pt, cte = self.get_regulated_lookahead(cur_x, cur_y, dist_to_goal)
                self.cte_pub.publish(Float32(data=cte))

                # -------------------------------------------------------------
                # Entry Mode: ใกล้สถานีแล้ว -> สลับเป้าไป "จุดเช็ค" (station - 1m ตาม yaw จาก CSV)
                # -------------------------------------------------------------
                entry_pt = None
                entry_mode = False
                if self.target_station_data is not None and dist_to_goal <= self.entry_trigger_dist:
                    tau = self.station_yaw(self.target_station_data)
                    # เลือกฝั่งจุดเช็ค: หุ่นอยู่ระหว่างทางเข้าตามแกน -> จุดเช็คอยู่ฝั่งเดียวกับหุ่น
                    # ถ้าเกือบตั้งฉาก (เข้าโค้ง 90 องศา) ให้ยืด "หลังสถานีตาม yaw" (st - ê) ซึ่งเป็นทิศหันหน้าปกติ
                    dot = ((cur_x - self.target_station_data['x']) * math.cos(tau)
                           + (cur_y - self.target_station_data['y']) * math.sin(tau))
                    s = 1.0 if dot > 0.25 else -1.0
                    entry_pt = Point(
                        self.target_station_data['x'] + s * math.cos(tau) * self.entry_check_dist,
                        self.target_station_data['y'] + s * math.sin(tau) * self.entry_check_dist,
                        0.0)
                    entry_mode = True

                # จุดตัดเข้า STATION_ALIGN: ถึงจุดเช็ค (หรือระยะจอดเดิม ถ้าไม่รู้สถานี)
                if entry_mode:
                    dist_entry = math.hypot(entry_pt.x - cur_x, entry_pt.y - cur_y)
                    dist_st_c = math.hypot(self.target_station_data['x'] - cur_x, self.target_station_data['y'] - cur_y)
                    at_entry = dist_entry <= self.entry_hold_dist or dist_st_c <= self.station_dist_tol
                else:
                    at_entry = dist_to_goal <= self.station_dist_tol

                if at_entry:
                    # Debounce: ต้องอยู่ข้างในเกณฑ์ต่อเนื่องหลายลูป -> กัน AMCL/TF เด้งแล้วจอดหลอก
                    self.goal_confirm_count += 1
                    if self.goal_confirm_count >= self.goal_confirm_loops:
                        self.current_v = 0.0
                        cmd.linear.x, cmd.angular.z = 0.0, 0.0
                        self.cmd_pub.publish(cmd)
                        if entry_mode:
                            rospy.loginfo(f"[Dock] Align start: {dist_entry:.2f} m from checkpoint / {dist_st_c:.2f} m from station.")
                        self.state = 'STATION_ALIGN'
                    else:
                        cmd.linear.x, cmd.angular.z = 0.0, 0.0
                elif target_pt:
                    self.goal_confirm_count = 0

                    if entry_mode:
                        # ช่วง "ลัดเข้าจุดเช็ค": เป้า = จุดเช็ค ตรงๆ (ใช้โค้งบิดเข้าตาม PP)
                        target_pt = entry_pt
                        dx = target_pt.x - cur_x
                        dy = target_pt.y - cur_y
                        alpha = math.atan2(math.sin(math.atan2(dy, dx) - cur_yaw), math.cos(math.atan2(dy, dx) - cur_yaw))
                    elif dist_to_goal <= self.lookahead_dist and len(self.path_poses) >= 3:
                        # ตอนใกล้เข้าจอด ให้ไล่มุมตามทิศทางช่วงสุดท้ายของ path แทนการเล็งจุดสุดท้าย
                        # (จุดสุดท้ายระยะถิด มุมหน้าง่ายๆ กระโดดตาม AMCL noise -> หัวสั่น)
                        seg_yaw = math.atan2(
                            goal_p.y - self.path_poses[-3].pose.position.y,
                            goal_p.x - self.path_poses[-3].pose.position.x)
                        alpha = math.atan2(math.sin(seg_yaw - cur_yaw), math.cos(seg_yaw - cur_yaw))
                    else:
                        dx = target_pt.x - cur_x
                        dy = target_pt.y - cur_y
                        alpha = math.atan2(math.sin(math.atan2(dy, dx) - cur_yaw), math.cos(math.atan2(dy, dx) - cur_yaw))

                    # 1. RPP Curvature Regulation (ยิ่งเบี่ยง ยิ่งต้องช้า)
                    v_curve = self.max_lin_vel / (1.0 + 1.5 * abs(alpha))

                    # 2. RPP Approach Deceleration (ยิ่งใกล้เป้า ยิ่งต้องช้า)
                    v_approach = self.max_lin_vel
                    if dist_to_goal < self.decel_dist_station:
                        v_approach = self.min_lin_vel + (self.max_lin_vel - self.min_lin_vel) * (dist_to_goal / self.decel_dist_station)

                    # 2.5 Entry Mode cap ความเร็วช่วงลัดเข้าจุดเช็ค
                    if entry_mode:
                        v_approach = min(v_approach, self.entry_approach_v)

                    # เลือกความเร็วที่ต่ำที่สุด และไม่ต่ำกว่าจุดคลาน (Creep)
                    target_v = max(self.min_lin_vel, min(v_curve, v_approach))

                    # 3. Acceleration Ramp
                    if target_v > self.current_v:
                        self.current_v = min(target_v, self.current_v + self.max_accel * dt)
                    else:
                        self.current_v = max(target_v, self.current_v - self.max_decel * dt)

                    # 4. RPP Kinematic Steering (บีบหน้าเลี้ยวให้สัมพันธ์กับความเร็ว)
                    # กันอาการส่ายตอนความเร็วต่ำ โดย Clamp มุมเลี้ยวตามความเร็ว (บังคับมุมขั้นต่ำไว้ 0.30 rad)
                    speed_factor = min(1.0, max(0.15, self.current_v / self.max_lin_vel))
                    steer_limit = 0.30 + 0.35 * speed_factor
                    clamped_alpha = max(min(alpha, steer_limit), -steer_limit)

                    effective_ld = max(0.35, min(self.lookahead_dist, dist_to_goal))
                    omega = self.current_v * ((2.0 * math.sin(clamped_alpha)) / effective_ld)

                    cmd.linear.x = self.current_v
                    cmd.angular.z = max(min(omega, self.max_ang_vel), -self.max_ang_vel)

            # -------------------------------------------------------------
            # STATE 3: STATION_ALIGN (หมุนจัดหน้า + creep หน้าตรงเข้าจอด)
            # -------------------------------------------------------------
            elif self.state == 'STATION_ALIGN':
                self.status_pub.publish(String(data="STATION_ALIGN"))

                if self.target_station_data:
                    st_x = self.target_station_data['x']
                    st_y = self.target_station_data['y']
                    qz, qw = self.target_station_data['qz'], self.target_station_data['qw']
                    st_name, mode, timer = self.target_station_data['name'], self.target_station_data['mode'], self.target_station_data['timer']
                else:
                    st_x = self.path_poses[-1].pose.position.x
                    st_y = self.path_poses[-1].pose.position.y
                    last_rot = self.path_poses[-1].pose.orientation
                    qz, qw = last_rot.z, last_rot.w
                    st_name, mode, timer = "Target", "Auto", 0.0

                # หมุนตามค่าในไฟล์ CSV ตรงๆ (ยกเลิก 180 องศา U-Turn)
                (_, _, target_yaw) = tf.transformations.euler_from_quaternion([0.0, 0.0, qz, qw])
                yaw_err = math.atan2(math.sin(target_yaw - cur_yaw), math.cos(target_yaw - cur_yaw))
                dist_st = math.hypot(st_x - cur_x, st_y - cur_y)
                des_yaw = math.atan2(st_y - cur_y, st_x - cur_x)
                yaw_to_st = math.atan2(math.sin(des_yaw - cur_yaw), math.cos(des_yaw - cur_yaw))

                if abs(yaw_to_st) > 0.30 and dist_st > 0.20:
                    # เฟส B: หมุนอยู่กับที่ให้หัน "เข้าหาศูนย์กลางสถานี" ก่อน (ไม่หมุนตาม CSV yaw ตรงๆ
                    # เพราะหุ่นอาจเข้ากับทางเบื้องหลัง -> เดี๋ยวหมุนกลับหลัง) ควบคุมแบบ Hysteresis
                    self.current_v = 0.0
                    cmd.linear.x = 0.0
                    cmd.angular.z = self.direct_rotate(yaw_to_st)
                elif dist_st > self.dock_align_tol:
                    # เฟส C: เลื่อนตรง 0.10 m/s เข้าหาศูนย์กลาง (แก้ค่า "ห่างจากแนว" อย่างหนึง ไม่กระโดด)
                    self.current_v = min(self.entry_linear_v, self.current_v + self.max_accel * dt)
                    cmd.linear.x = self.current_v

                    if abs(yaw_to_st) > 0.35 and dist_st > 0.25:
                        # ออกแนวมาก -> เล็งศูนย์กลางสถานี
                        cmd.angular.z = max(min(0.8 * yaw_to_st, 0.20), -0.20)
                    elif abs(yaw_to_st) > 0.12:
                        # ค่อนข้างตรง -> แก้มุมเบาๆ
                        cmd.angular.z = max(min(0.7 * yaw_to_st, 0.12), -0.12)
                    elif abs(yaw_err) > 0.06 and dist_st <= self.entry_check_dist:
                        # เหลือไม่ไกลนัก -> เริ่มแทรกองศาสถานีเข้าไปทีละน้อย (ใกล้=มาก)
                        cmd.angular.z = max(min(0.6 * yaw_err, 0.08), -0.08)
                    else:
                        cmd.angular.z = 0.0
                else:
                    # เฟส D: ถึงจุดจอด -> หมุนตามองศา CSV เป๊ะ (รวมเคสที่หมุน 180 เพื่อเทียบ) แล้วเทียบ
                    self.current_v = 0.0
                    cmd.linear.x = 0.0
                    if abs(yaw_err) > self.yaw_tol:
                        cmd.angular.z = self.direct_rotate(yaw_err)
                    else:
                        cmd.angular.z = 0.0
                        self.cmd_pub.publish(cmd)

                        if mode.lower() == 'manual':
                            rospy.loginfo(f"[Dock Complete] Reached [{st_name}]. Waiting for manual release...")
                            self.state = 'MANUAL_WAIT'
                        elif timer > 0.0:
                            self.wait_start_time = rospy.Time.now()
                            rospy.loginfo(f"[Dock Complete] Reached [{st_name}]. Waiting {timer}s...")
                            self.state = 'AUTO_WAIT'
                        else:
                            rospy.loginfo(f"[Dock Complete] Reached [{st_name}]. Ready.")
                            self.state = 'IDLE'

            # -------------------------------------------------------------
            # STATE 4: WAIT
            # -------------------------------------------------------------
            elif self.state == 'AUTO_WAIT':
                self.status_pub.publish(String(data="STOP_AT_STATION"))
                self.current_v = 0.0
                cmd.linear.x, cmd.angular.z = 0.0, 0.0
                
                if self.target_station_data:
                    elapsed = (rospy.Time.now() - self.wait_start_time).to_sec()
                    if elapsed >= self.target_station_data['timer']:
                        rospy.loginfo(f"[Auto Wait] Completed holding at [{self.target_station_data['name']}].")
                        self.state = 'IDLE'
                else:
                    self.state = 'IDLE'

            elif self.state == 'MANUAL_WAIT':
                self.status_pub.publish(String(data="WAIT_MANUAL_RELEASE"))
                self.current_v = 0.0
                cmd.linear.x, cmd.angular.z = 0.0, 0.0

            self.cmd_pub.publish(cmd)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        controller = RegulatedWaypointController()
        controller.run()
    except rospy.ROSInterruptException:
        pass