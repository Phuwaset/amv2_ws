# ปัญหาที่พบจากการตรวจสอบระบบ AMV2 (Problem Log)

**ไฟล์:** PROBLEM-LOG-001
**วันที่:** 2026-08-31
**วิธีตรวจ:** เช็คผ่าน ROS live data, rosparam, launch files, log systemd

| # | หมวด | ปัญหา | หลักฐาน / จุดที่พบ | สถานะ |
|---|---|---|---|---|
| 1 | Time | ค่า `use_sim_time: true` ตกค้างบนเครื่อง msi ทำให้ ros time หยุดรอ `/clock` → rviz error | env ของ msi (แก้โดย `rosparam set use_sim_time false`) | แก้แล้ว |
| 2 | Time | Chrony ไม่สามารถ sync เวลากับ msi ได้ (`Reach 0`, `LastRx -`) | `/etc/chrony/chrony.conf` ตั้ง `server 192.168.50.115` แต่ `chronyc sources` เห็น msi-wifi stratum 0, reach 0 | รอแก้ไข |
| 3 | Control | `twist_mux`: navigation priority = 10 (ต่ำสุด) ขณะที่ joystick/tablet = 100, keyboard = 90 → แตะจอยตอน auto จะแทรกแซงการนำทาง | `twist_mux_topics.yaml` (navigation=10, keyboard=90, joystick=100, tablet=100) | รอแก้ไข |
| 4 | Control | `twist_mux` timeout ของ nav_vel = 0.5s → move_base สะดุด >0.5s จะได้ความเร็ว 0 ทันที | `twist_mux_topics.yaml` | รอแก้ไข |
| 5 | Control | ไม่มีกลไก lock ตัด joystick ในโหมด auto (`joy_priority`, `pause_navigation` priority = 100) | `twist_mux_locks.yaml` | รอแก้ไข |
| 6 | Sensing | Lidar ครอบคลุมไม่เต็ม 360°: blind 60° หน้าเฉียงซ้าย/ขวา (30°+30°) | URDF `amv.urdf:73` (laser_front rpy -0.5236 = -30°) + `lidar_front.launch` (angle_min=-30, angle_max=90) → ครอบแค่ -60°..+60° ฝั่งหน้า, rear 90°..270° | รอแก้ไข |
| 7 | Sensing | `/laser_front` มีค่า 0.0 (no return) ถึง ~67% ของรังสีในหน้าต่าง 120° ที่ valid อยู่แค่ 0.25–0.47m | rostopic echo /laser_front (2020 rays, zeros=1362) | รอแก้ไข |
| 8 | Sensing | amcl `transform_tolerance = 0.2s` ค่อนข้างต่ำ เสี่ยง fail เมื่อ EKF/odom สะดุด | `amv_navigation/launch/amcl.launch:48` | แก้แล้ว (31-08-2026: = 0.5s — มีผลหลัง restart amcl/stack) |
| 9 | Sensing | `/scan` merger ตั้ง `scan_time = 0.0333` (30Hz) แต่ lidar จริง publish 10Hz → metadata ผิด | `ira_laser_tools/launch/laserscan_multi_merger.launch` | แก้แล้ว (31-08-2026: = 0.1, ตั้งผ่าน dynparam live แล้ว /scan ยืนยัน 0.1) |
| 10 | Resilience | โหนดหลักทุกตัวตั้ง `respawn="false"` → โหนด crash แล้วไม่ restart เอง | `lidar_front.launch`, `lidar_back.launch`, `amv_base_connect.launch` | รอแก้ไข |
| 11 | Resilience | Base node (`amv_base_node_28082026.py`) ไม่มี watchdog/reconnect สำหรับ serial `/dev/amv_controller` → USB หลุดแล้ว node ตาย | `amv_base_node_28082026.py` | รอแก้ไข |
| 12 | Sensing | `angular_velocity`/`linear_acceleration` ส่งเป็น 0 — code อ่าน GYRO frame แล้วแล้ว แต่เซนเซอร์ส่ง data gyro = `[0,0,0]` จริง (ตรวจ 05-09-31: raw 0x52 = 0,0,0 ทุกเฟรม) | `imu_df_connect.py` (อ่าน gyro แล้ว) / ต้องเช็ค register output content (0x30) ฝั่งเซนเซอร์ | รอแก้ไข |
| 13 | Sensing | IMU output 10Hz (สั่ง `FREQUENCY_50HZ` ถูก comment), baud 9600 | `imu_df_connect.py:57`, `imu_df_connect.py:40` | แก้แล้ว (31-08-2026: 20Hz @ 115200 ยืนยันแล้ว) |
| 14 | Sensing | EKF พับลิช `/odom` เป็น `geometry_msgs/PoseWithCovarianceStamped` (remap `odom_combined->odom`) → move_base drop connection, navigation ไม่มี odom | `robot_pose_ekf.launch` | แก้แล้ว (31-08-2026: เพิ่ม `odom_bridge.py` แปลง `odom_combined` เป็น `nav_msgs/Odometry` ที่ `/odom` ~25Hz, move_base เชื่อมต่อแล้ว) |

**หมายเหตุ:** รายละเอียดการแก้ไข/แนวทาง จะเพิ่มเติมภายหลังตามลำดับความสำคัญ (Tier 1: เวลา/การควบคุม → Tier 2: การรับรู้ → Tier 3: ความทนทาน)