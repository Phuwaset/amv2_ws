Autonomous Mobile Robot (AMV2) Navigation & Control Packages.
---

## 📊 ROS Computation Graph
ผังการเชื่อมต่อโหนดและ Topics ทั้งหมดของระบบ AMV2 ขณะทำงานผ่าน `amv-start.service`:

<p align="center">
  <img src="amv_navigation/rosgraph_amv2.png" alt="AMV2 ROS Computation Graph" width="100%">
</p>

---

### Core Packages
* `amv_connect`: บอร์ดสื่อสารฮาร์ดแวร์, WebSocket bridge และสถานะไฟ/เสียง
* `amv_navigation`: ระบบนำทางหลัก (Costmap, AMCL, Global/Local Planner)
* `amv_service`: จัดการสถานี ภารกิจ และ Waypoint Control
* `amv_virtual_track`: ระบบรางนำทางเสมือน (Virtual Track Controller)
* `ydlidar_ros-master` & `ira_laser_tools`: ไดรเวอร์และการรวมสัญญาณ LiDAR หน้า-หลัง
