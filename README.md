Autonomous Mobile Robot (AMV2).
---

## ROS Computation Graph
ผังการเชื่อมต่อโหนดและ Topics ทั้งหมดของระบบ AMV2 ขณะทำงานผ่าน `amv-start.service` (ดึงจาก rqt_graph แบบสด):

<p align="center">
  <img src="amv_navigation/rosgraph_amv2.png" alt="AMV2 ROS Computation Graph" width="100%">
</p>

**วิธีอ่านกราฟ:** วงรี = ROS Node (โปรเซส), สี่เหลี่ยม = Topic (ช่องส่งข้อมูล), กล่องกลุ่ม `move_base` = namespace ย่อยของโหนด, `/tf` = topic การแปลงพิกัดที่ถูก Group รวมเป็นโหนดเดียว

---

### 1. Data Flow (เส้นทางข้อมูลหลัก)

```
[LiDAR หน้า/หลัง] ──/laser_front, /laser_back──> [/laserscan_multi_merger] ──/scan──> [/amcl], [/laser_safety_zone1]
                                                                                              │ /zone1 (สั่งหยุดฉุกเฉิน)
[IMU] ──/imu/data──> [/imu_df_node] ──/imu──> [/robot_pose_ekf]
[/amv_model] ──/tf──> [/robot_pose_ekf] ──/robot_pose_ekf/odom_combined──> [/ekf_odom_bridge] ──/odom──> [/move_base]

[/map_server] ──/map──> [/move_base], [/amcl]          (แผนที่โลจิสติก)
[/amcl] ──/amcl_pose + /tf──> [/move_base]             (ตำแหน่งหุ่นที่ estimate ได้)

[/station_service] ──/move_base_simple/goal──> [/move_base]  (รับเป้าหมายจาก Web App / สถานี)
[/move_base] ──/nav_vel──> [/twist_mux] ──/set_velocity──> [/amv_base]  (คำสั่งเคลื่อนที่ลงบอร์ด)

[Joystick] ──/joy──> [/joy_to_twist] ──/joy_vel──> [/twist_mux]  (ควบคุมด้วยมือ ชนะ Auto เมื่อบังคับ)
```

### 2. โหนดหลักและบทบาท

| โหนด | แพ็กเกจ | บทบาท |
|---|---|---|
| `/ydlidar_front`, `/ydlidar_back` | `ydlidar_ros` | ไดรเวอร์ LiDAR หน้า/หลัง |
| `/laserscan_multi_merger` | `ira_laser_tools` | รวมสแกนหน้า+หลังเป็น `/scan` เดียว |
| `/laser_safety_zone1` | `amv_connect` | กันชนเสมือน (Safety Zone) — สั่งหยุด/ลดความเร็วผ่าน `/zone1` |
| `/imu_df_node` | `amv_connect` | กรอง/ประมวลผล IMU (Dead Reckoning) |
| `/robot_pose_ekf` | `robot_pose_ekf` | ฟิวชัน Odometry + IMU เป็น `/tf` ของหุ่น |
| `/ekf_odom_bridge` | `amv_connect` | ปรับรูป `/robot_pose_ekf/odom_combined` → `/odom` ให้ move_base |
| `/map_server` | `map_server` | โหลดแผนที่ `office_test_v3` |
| `/amcl` | `amcl` | Monte Carlo Localization — ประมาณตำแหน่งบนแผนที่ |
| `/move_base` | `move_base` | Global (PlannedPathPlanner) + Local (DWA) Planner, costmap 2 ชั้น |
| `/amv_base` | `amv_connect` | สื่อสารบอร์ดควบคุม (`/dev/amv_controller`), publish `/odom_amv` |
| `/amv_current_pose`, `/amv_model`, `/amv_pose_tf` | `amv_connect` | ประมวลผล/แปลงตำแหน่งสำหรับ UI และ TF |
| `/twist_mux`, `/twist_marker`, `/joy_to_twist` | `twist_mux` | ตัวสลับคำสั่งความเร็ว (Auto / Joy) + แสดงภาพ cmd_vel |
| `/station_service` | `amv_service` | จัดการสถานี ภารกิจ Waypoint (`/pin_command`, `/led_command`) |
| `/station_plotting` | `amv_service` | จุด plot เส้นทาง/ภารกิจบนแผนที่ |
| `/rosbridge_server` | `rosbridge_server` | WebSocket ให้ Web App ควบคุมผ่าน `/amv_pose_str` ฯลฯ |
| `/rviz_*` | `rviz` | Viewer (ไม่กระทบการทำงาน) |

### 2.1 move_base และส่วนประกอบ (Planner / Costmap / Topics)

| ส่วน | ทำหน้าที่ | Topic |
|---|---|---|
| `/move_base` (node) | รับเป้าหมายผ่าน action server + state machine (PLANNING → CONTROLLING → CLEARING); ส่งคำสั่งความเร็วผ่าน remap `cmd_vel` → `/nav_vel` | `/move_base/status`, `/move_base/current_goal`, `/move_base/goal` |
| `base_global_planner` = `amv_navigation/PlannedPathPlanner` | Global planner ตัวหลัก: เกาะเส้น `/planned_path` ที่วางจาก `path_planner_node` (amv_virtual_track, Dijkstra routing) แล้วคืน plan ให้ move_base | รับ `/planned_path` — พิมพ์ `/move_base/PlannedPathPlanner/plan` (เส้นสีเขียวใน RViz) |
| fallback `navfn_fallback` | NavfnROS ที่ฝังอยู่ภายใน planner — ใช้เมื่อยังไม่มี `/planned_path` หรือ start/goal อยู่นอกเส้นเกิน `planned_path_max_match_dist` (1.0 m) | `/move_base/navfn_fallback/plan` |
| `base_local_planner` = `dwa_local_planner/DWAPlannerROS` | Local planner: DWA สุ่มวิถีความเร็ว (vx/vth) เลือกคำสั่งที่ดีที่สุด; พิมพ์ plan ที่ prune แล้ว + local plan + วิถีตัวอย่าง | `/move_base/DWAPlannerROS/global_plan`, `/move_base/DWAPlannerROS/local_plan`, `/move_base/DWAPlannerROS/trajectory_cloud` |
| `/move_base/global_costmap` | Costmap ระดับโลก: static map + inflation + `costmap_prohibition_layer` | `/move_base/global_costmap/costmap` |
| `/move_base/local_costmap` | Costmap ระดับท้องถิ่น: static + observation (LiDAR) + inflation | `/move_base/local_costmap/costmap` |
| รับเป้าหมาย | จาก Web App / `station_service` ผ่าน topic | `/move_base_simple/goal` |
| Recovery behavior | ปิดใช้งาน: `recovery_behavior_enabled=false`, `clearing_rotation_allowed=false` | `/move_base/recovery_status` |

**หมายเหตุ:**
- `/move_base/NavfnROS/plan` ไม่มีแล้ว — เดิม global planner เป็น `navfn/NavfnROS` ตรงๆ ตอนนี้ใช้ `amv_navigation/PlannedPathPlanner` (`move_base.launch` บรรทัด 12)
- เส้นแผนที่ใน RViz (`navigation.rviz`) แสดงจาก `/move_base/PlannedPathPlanner/plan` (สีเขียว = global plan) และ `/move_base/DWAPlannerROS/local_plan` (สีแดง = local plan)

### 3. Core Packages
* `amv_connect`: บอร์ดสื่อสารฮาร์ดแวร์, WebSocket bridge และสถานะไฟ/เสียง
* `amv_navigation`: ระบบนำทางหลัก (Costmap, AMCL, Global/Local Planner)
* `amv_service`: จัดการสถานี ภารกิจ และ Waypoint Control
* `amv_virtual_track`: ระบบรางนำทางเสมือน (Virtual Track Controller) — **ยังไม่เปิดใช้งานใน service**
* `ydlidar_ros-master` & `ira_laser_tools`: ไดรเวอร์และการรวมสัญญาณ LiDAR หน้า-หลัง

> **หมายเหตุ:** กราฟนี้เป็นสถานะที่รันจริงผ่าน `amv-start.service` (เปิดเฉพาะ `amv_navigation.launch`); `amv_qr_detection` ถูกคอมเมนต์ไว้ใน launch และ `amv_mapping` ใช้เฉพาะตอนทำแผนที่เท่านั้น


After mapping Go to Step2

Step2

* Stop service robot

sudo systemctl stop amv-start.service

* test offine amv-start.service for recoding 
roslaunch amv_navigation amv_recording.launch

* Manual recodr station with 2d estimate
rosrun amv_service station_plotting_test.py

* Manual recodr waypoint with 2d nav_goal
rosrun amv_service waypoint_recorder_test.py


Service	ทำอะไร
rosservice call /amv/recorder/undo_station	ลบสถานีล่าสุด + เขียน CSV + วาด marker ใหม่
rosservice call /amv/recorder/clear_station	ล้างสถานีทั้งหมด
rosservice call /amv/recorder/save_station	บันทึก CSV


rosservice call /amv/recorder/undo_waypoint     # ลบ waypoint ล่าสุด + save CSV + วาด marker ใหม่
rosservice call /amv/recorder/clear_waypoint     # ล้างทั้งหมด
rosservice call /amv/recorder/save_waypoint      # save (บันทึกอัตโนมัติอยู่แล้ว)




มิติการเปรียบเทียบ/planned_path/move_base/PlannedPathPlanner/planโหนดต้นทางpath_planner_node.py  โหนด move_base (ปลั๊กอิน PlannedPathPlanner)  ขอบเขตเส้นทางแนวทางเดินทั้งหมดของภารกิจ (Material Room $\rightarrow$ wy1 $\rightarrow$ wy2 $\rightarrow$ Line1)ตัดเฉพาะช่วงจาก "ตำแหน่งที่รถอยู่ปัจจุบัน" มุ่งหน้าไปหา Goal  ความสัมพันธ์กับ Costmapไม่สนใจ Costmap (คำนวณตาม Topological Graph ใน CSV)  เชื่อมโยงกับ global_costmap เพื่อตรวจสอบสิ่งกีดขวางและอนุญาตให้ Fallback  ปลายทางผู้ใช้งานแสดงผลบน RViz และส่งให้ปลั๊กอิน Planner ดึงไปอ้างอิง  ส่งตรงเข้า Local Planner (DWAPlannerROS) เพื่อคำนวณความเร็วขับล้อ  สถานะเมื่อรถหลุดนอกแนวคงตำแหน่งเส้นเดิมไว้ ไม่เปลี่ยนรูปร่างหากรถอยู่ห่างเกิน $1.0\text{ m}$ จะสลับรูปร่างกลายเป็นเส้นทางของ Navfn อัตโนมัติ  



ชั้นที่ 1 (ภายนอก): path_planner_node.py ปล่อย /planned_path (เส้นเขียวทั้งสายจาก CSV)ชั้นที่ 2 (Global Planner): PlannedPathPlanner ดึงมาตัดเป็น /move_base/PlannedPathPlanner/plan (เส้นจากจุดที่รถยืนอยู่ $\rightarrow$ ปลายทาง)ชั้นที่ 3 (Local Planner Reference): DWAPlannerROS ดึงไปครอบด้วย Local Costmap กลายเป็น /move_base/DWAPlannerROS/global_plan (ช่วง 3 เมตรข้างหน้ารถ)ชั้นที่ 4 (Trajectory Generation): DWAPlannerROS สุ่มจำลองวิถีขับเคลื่อนจริงออกมาเป็น /move_base/DWAPlannerROS/local_plan แล้วแปลงเป็นคำสั่งส่งลงล้อทางเลือกสำรอง (Safety Fallback): หากรถหลุดรางเกิน 1.0 เมตร ระบบจะตัดการทำงานไปใช้ /move_base/navfn_fallback/plan นำทางแบบพื้นที่อิสระแทนทันที



Test pub path planner
rostopic pub -1 /amv/command/goto_station std_msgs/String "data: 'Line1'"

Test pub goto_station
rosservice call /go_station "target_station: 'Line1'"