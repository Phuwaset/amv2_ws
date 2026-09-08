# คู่มือมาตรฐานการปฏิบัติงาน (SOP Manual)

## รหัสเอกสาร: SOP-ROBOT-TIME-001

**ชื่อเอกสาร:** คู่มือการบริหารจัดการระบบเวลา (ROS Time Architecture), การตั้งค่า `use_sim_time` และการแก้ไขปัญหา TF Extrapolation Error

**ระบบที่รองรับ:** ROS 1 Noetic / Ubuntu 20.04 LTS (หุ่นยนต์ AMV2 / Multi-Machine Network / Gazebo Simulator)

---

## สารบัญหัวข้อเอกสาร (Table of Contents)

- **หมวดที่ 1: สถาปัตยกรรมระบบเวลาใน ROS 1 (ROS Time Architecture)**
  - 1.1 ความแตกต่างระหว่าง Wall Time (System Clock) กับ ROS Time
  - 1.2 กลไกการทำงานของ Parameter `/use_sim_time`
  - 1.3 บทบาทของ Topic `/clock` ในระบบ Simulation (Gazebo)
  - 1.4 การเปรียบเทียบสถานะโหมดจำลอง (Sim) vs โหมดหุ่นยนต์จริง (Physical Robot)

- **หมวดที่ 2: ถอดรหัสปัญหาและอาการแจ้งเตือน (Symptom Analysis)**
  - 2.1 ทำความเข้าใจข้อผิดพลาด `TF Error: Lookup would require extrapolation into the future`
  - 2.2 ทำไม RViz จึงแสดงสถานะสีแดง (Error) ที่ LaserScan หรือ Fixed Frame
  - 2.3 การเปรียบเทียบเวลา: Sensor Timestamp (`Requested time`) vs TF Buffer (`Latest data`)
  - 2.4 ตารางวิเคราะห์ขนาดความเหลื่อมของเวลา (Δt) เพื่อจำแนกต้นตอ

- **หมวดที่ 3: บทเรียน Case Study และกับดักทางเทคนิคที่ทำให้หลงทาง (Debugging Pitfalls)**
  - 3.1 กับดักที่ 1: ปัญหา Clock Drift ข้ามเครื่อง (`amv2` vs `msi`) ในเครือข่าย Wi-Fi วงปิด
  - 3.2 กับดักที่ 2: การปรับ `transform_tolerance` บนโหนด AMCL
  - 3.3 กับดักที่ 3: อัตรา EKF ตกฮวบเหลือ 1 Hz จากชื่อ Topic IMU ไม่ตรง (`/imu_data` vs `/imu/data`)
  - 3.4 กับดักที่ 4: ความหน่วงสะสมใน Serial Buffer และโหนดรวมเลเซอร์ (`laserscan_multi_merger`)
  - 3.5 ต้นตอที่แท้จริง: ค่า `use_sim_time: true` ตกค้างใน Parameter Server

- **หมวดที่ 4: ขั้นตอนการตรวจเช็คตามลำดับ (Standard Troubleshooting Workflow)**
  - Step 1: ตรวจสอบสถานะ `use_sim_time` และ Topic `/clock`
  - Step 2: ตรวจสอบความสอดคล้องของเวลาระบบ (Host System Clock Check)
  - Step 3: ตรวจสอบความถี่และ Timestamp ของเซ็นเซอร์ต้นทาง (`/scan`, `/odom_amv`, `/imu/data`)
  - Step 4: ตรวจสอบสายโซ่พิกัด TF Chain ทีละท่อนด้วย `tf_echo`
  - Step 5: ตรวจสอบการปล่อยพิกัดของ AMCL (`map -> odom`) และการป้อน Initial Pose

- **หมวดที่ 5: ชุดคำสั่งด่วนและสคริปต์ตรวจสอบความพร้อม (Commands & Cheat Sheet)**
  - 5.1 รวมคำสั่งตั้งค่าและตรวจสอบค่า Parameter
  - 5.2 สคริปต์ Pre-flight Check ตรวจสอบระบบเวลาก่อนเปิดระบบขับเคลื่อนอัตโนมัติ

- **หมวดที่ 6: แนวทางการป้องกันปัญหาถาวร (Architectural Best Practices)**
  - 6.1 การแยกโครงสร้าง Launch Files สำหรับ Simulation และ Hardware ชัดเจน
  - 6.2 การตั้งค่า Chrony Local NTP Master-Client ในเครือข่ายออฟไลน์
  - 6.3 การจัดการ Systemd Service (`amv-start.service`) ให้รีเซ็ตสภาพแวดล้อมอย่างปลอดภัย

---

## รายละเอียดเนื้อหาในแต่ละหมวด

### หมวดที่ 1: สถาปัตยกรรมระบบเวลาใน ROS 1 (ROS Time Architecture)

#### 1.1 สองแกนเวลาใน ROS

ใน ROS 1 มีการอ้างอิงเวลา 2 รูปแบบ:

1. **Wall Time (เวลานาฬิกาจริง):** ดึงเวลาตรงจาก Hardware RTC และ Linux Kernel ของเครื่องคอมพิวเตอร์นั้น ๆ ผ่านฟังก์ชันระบบปฏิบัติการ (Unix Epoch Timestamp เช่น `178791xxxx`)

2. **ROS Time (เวลาภายในระบบ ROS):** เวลาที่เรียกผ่านฟังก์ชัน `ros::Time::now()` หรือ `rospy.Time.now()` ซึ่งพฤติกรรมจะขึ้นอยู่กับพารามิเตอร์ `/use_sim_time`

#### 1.2 บทบาทของ Parameter `/use_sim_time`

- **เมื่อตั้งค่าเป็น `false` (โหมดหุ่นยนต์จริง):**
  ฟังก์ชัน `ros::Time::now()` จะดึงเวลาตรงจาก System Clock ของเครื่อง Linux โดยอัตโนมัติ ทำให้ทุกเซ็นเซอร์และโหนดคำนวณ TF ทำงานบนเวลาจริงของบอร์ด

- **เมื่อตั้งค่าเป็น `true` (โหมดจำลองสถานการณ์):**
  `ros::Time::now()` จะ **หยุดอ่านนาฬิกาของเครื่อง Linux ทันที** และจะรอรับแพ็กเก็ตเวลาจำลองที่ถูกส่งเข้ามาทาง Topic `/clock` (ประเภท `rosgraph_msgs/Clock`) เท่านั้น

#### 1.3 ตารางเปรียบเทียบโหมดการทำงาน

| สภาพแวดล้อม | ค่า `use_sim_time` | แหล่งกำเนิดเวลา (Time Source) | พฤติกรรมเมื่อไม่มีตัวส่งเวลา |
| --- | --- | --- | --- |
| **Gazebo Simulation** | `true` | Gazebo Physics Engine Broadcast ลง Topic `/clock` | ทุกโหนด (TF, AMCL, RViz) จะหยุดนิ่งรอสัญญาณนาฬิกา |
| **หุ่นยนต์จริง (AMV2)** | `false` | Linux Kernel System Clock ของบอร์ด `amv2` | ทำงานตามเวลาจริงต่อเนื่อง ไม่รอ Topic `/clock` |

---

### หมวดที่ 2: ถอดรหัสปัญหาและอาการแจ้งเตือน (Symptom Analysis)

#### 2.1 ข้อความเตือน `Lookup would require extrapolation into the future`

เมื่อดู Log จาก RViz หรือ Terminal:

```text
For frame [laser_link]: No transform to fixed frame [odom].
TF error: [Lookup would require extrapolation -23.398732653s into the future.
Requested time 1787908244.308048010 but the latest data is at time 1787908220.909315348...]
```

- **Requested time:** เวลาที่เซ็นเซอร์ประทับมาในหัวข้อความ (Header Stamp ของ `/scan` หรือ `/laser_front`)
- **Latest data:** เวลาล่าสุดของเฟรมพิกัดที่มีอยู่ในหน่วยความจำชั่วคราว (TF Buffer) ของระบบ
- **ผลต่างของเวลา (Δt):** ตัวเลขติดลบ เช่น `-23.39s` หมายถึง **TF ล่าสุดในระบบมีข้อมูลช้ากว่าเวลาของข้อมูลเซ็นเซอร์ไปถึง 23 วินาที** ในระบบ ROS TF จะเก็บบัฟเฟอร์ย้อนหลังไว้เพียง ~10 วินาที และจะปฏิเสธการคำนวณแปลงพิกัดทันทีหากถูกขอให้ทำนายตำแหน่งในอนาคต (Extrapolation)

#### 2.2 ตารางวินิจฉัยขนาดของผลต่างเวลา (Δt)

| ขนาดความต่าง (Δt) | สาเหตุที่เป็นไปได้สูงที่สุด | จุดที่ต้องเข้าไปตรวจสอบ |
| --- | --- | --- |
| **0.05 – 0.30 วินาที** | ความหน่วงจากการคำนวณของ Filter (EKF Lag) หรือความถี่ Publish TF ต่ำเกินไป | ตรวจสอบพารามิเตอร์ `freq` ใน `robot_pose_ekf` และการ Remap IMU Topic |
| **0.50 – 5.00 วินาที** | เวลาของคอมพิวเตอร์ 2 เครื่องเดินไม่เท่ากัน (Clock Drift ในวง Wi-Fi ปิด) | ตรวจสอบคำสั่ง `date` ระหว่าง Host กับ Robot และตั้งค่า Chrony |
| **> 10.00 วินาทีขึ้นไป หรือค้างถาวร** | **`use_sim_time: true` ค้างอยู่** หรือโหนดสร้าง TF ตาย/ค้าง | ตรวจสอบ `rosparam get /use_sim_time` และเช็คโหนดด้วย `tf_echo` |

---

### หมวดที่ 3: บทเรียนและกับดักทางเทคนิคที่ทำให้หลงทาง (The Debugging Pitfalls)

จากเหตุการณ์ที่ใช้เวลาแก้ไขยาวนาน เกิดจากการมีหลายปัจจัยซ้อนทับกันจนบดบังต้นตอหลัก:

1. **กับดักเรื่อง Clock Drift ข้ามเครื่อง (`msi` vs `amv2`):**
   เนื่องจากหุ่นยนต์ทำงานในวง Wi-Fi ที่ไม่มีอินเทอร์เน็ต นาฬิกา Hardware RTC จึงเหลื่อมกันจริง 2–5 วินาที ทำให้การซิงก์ด้วย `sudo date -s` ดูเหมือนเป็นทางแก้ แต่เมื่อซิงก์แล้ว Error ยังคงอยู่ เพราะ ROS Time ภายในยังค้าง

2. **กับดักเรื่อง `transform_tolerance` บน AMCL:**
   การพยายามขยายกรอบเวลารองรับ (Tolerance) จาก `0.2` เป็น `0.5` วินาที ไม่สามารถแก้ปัญหาได้ เพราะความคลาดเคลื่อนจริงสูงถึงระดับหลายสิบวินาที

3. **กับดักเรื่องความถี่ EKF ตกฮวบเหลือ 1 Hz:**
   `robot_pose_ekf` ปกติฟัง Topic ชื่อ `imu_data` (ไม่มี slash) แต่ไดรเวอร์ส่งมาที่ `/imu/data` โหนด EKF จึงรอจนครบ `sensor_timeout: 1.0s` แล้วค่อยปล่อย TF ทำให้ระบบกระตุกเป็นจังหวะรอบละ 1 วินาที

4. **กับดักเรื่อง Serial Delay & LiDAR Merger:**
   คำสั่ง `time.sleep(2)` ที่วางผิดตำแหน่งในตัวอ่าน Serial ทำให้ข้อมูลล้อชุดแรกค้างใน Buffer และตัวรวมเลเซอร์ `laserscan_multi_merger` ต้องรอ Timestamp ที่สอดคล้องกันของหัวเลเซอร์สองตัว

5. **จุดหักมุม (Root Cause):**
   การทดสอบ Gazebo ในช่วงเช้าได้บันทึกค่า `use_sim_time: true` ลงใน Parameter Server ไว้ เมื่อปิด Gazebo แล้วย้ายมารันบนหุ่นยนต์จริงโดยไม่ได้ Restart `roscore` หรือไม่ได้ตั้งค่ากลับเป็น `false` ทุกโหนดจึงรอคอย Topic `/clock` ที่ไม่มีอยู่จริง

---

### หมวดที่ 4: ขั้นตอนการตรวจเช็คตามลำดับ (Standard Troubleshooting Workflow)

```
[เริ่มต้นพบปัญหา Extrapolation Error]
                │
                ▼
  [Step 1] ตรวจสอบ use_sim_time
   rosparam get /use_sim_time ───(ได้ค่า true บนหุ่นจริง)───► rosparam set use_sim_time false
                │ (ได้ค่า false ถูกต้อง)
                ▼
  [Step 2] ตรวจสอบเวลาระบบ 2 เครื่อง
   date (บน Robot) เทียบกับ date (บน Host) ──(ต่างกัน > 0.2s)──► ซิงก์เวลาด้วย Chrony / date -s
                │ (เวลาตรงกัน)
                ▼
  [Step 3] ตรวจสอบ Timestamp ของ Sensor
   rostopic echo /scan/header/stamp/secs
   rostopic echo /odom_amv/header/stamp/secs ──(ต่างกันหลักวินาที)──► ตรวจสอบ Serial Buffer & Driver
                │ (เวลา Timestamp ตรงกัน)
                ▼
  [Step 4] ตรวจสอบสายโซ่ TF Chain
   rosrun tf tf_echo odom base_footprint ──(TF ไม่เดิน / ค้าง)──► เช็ค EKF Remap (/imu/data) & ความถี่
                │ (TF odom เดินปกติ)
                ▼
  [Step 5] ตรวจสอบโหนดนำทาง AMCL
   RViz Fixed Frame = map ──(map -> odom ไม่เดิน)──► ป้อน Initial Pose ผ่าน 2D Pose Estimate
```

---

### หมวดที่ 5: ชุดคำสั่งด่วนและสคริปต์ตรวจสอบ (Commands & Cheat Sheet)

#### 5.1 รวมคำสั่งตั้งค่าและตรวจสอบค่าเวลา

```bash
# 1. เช็คค่า use_sim_time ปัจจุบัน
rosparam get /use_sim_time

# 2. ตั้งค่าให้ถูกต้องตามโหมดการทำงาน
rosparam set use_sim_time false   # สำหรับหุ่นยนต์จริง amv2
rosparam set use_sim_time true    # สำหรับ Gazebo Simulation

# 3. ตรวจสอบว่ามีโหนดปล่อยเวลาจำลองหรือไม่
rostopic hz /clock

# 4. ตรวจสอบ Timestamp ของ Sensor เทียบกับเวลานาฬิกา Linux
date +%s && rostopic echo /scan/header/stamp/secs -n 1 && rostopic echo /odom_amv/header/stamp/secs -n 1

# 5. เช็คความต่อเนื่องของ TF ข้อต่อฐานล้อ
rosrun tf tf_echo odom base_footprint
```

#### 5.2 สคริปต์ Pre-flight Check ระบบเวลา (`check_ros_time.sh`)

นำสคริปต์นี้ไปวางบนเครื่องหุ่นยนต์เพื่อสั่งรันก่อนเริ่มนำทางอัตโนมัติ:

```bash
#!/bin/bash
echo "=== AMV2 ROS TIME INTEGRITY CHECK ==="

# 1. เช็ค use_sim_time
SIM_TIME=$(rosparam get /use_sim_time 2>/dev/null)
if [ "$SIM_TIME" == "true" ]; then
    echo "[CRITICAL ERROR] /use_sim_time is TRUE on Physical Robot!"
    echo "Fixing: Setting /use_sim_time to FALSE..."
    rosparam set use_sim_time false
else
    echo "[OK] /use_sim_time is correctly set to FALSE."
fi

# 2. เช็ค Topic /clock
CLOCK_CHECK=$(timeout 2 rostopic echo /clock -n 1 2>/dev/null)
if [ -n "$CLOCK_CHECK" ]; then
    echo "[WARNING] Topic /clock is active! A simulator node might be conflicting."
else
    echo "[OK] No phantom /clock topic detected."
fi

# 3. เช็คความแตกต่างของเวลาระหว่าง Sensors
echo "[INFO] Sampling Sensor Timestamps..."
SCAN_TIME=$(rostopic echo /scan/header/stamp/secs -n 1 2>/dev/null)
ODOM_TIME=$(rostopic echo /odom_amv/header/stamp/secs -n 1 2>/dev/null)
SYS_TIME=$(date +%s)

echo "System Clock : $SYS_TIME"
echo "LiDAR Scan   : $SCAN_TIME"
echo "Wheel Odom   : $ODOM_TIME"

DIFF=$((SCAN_TIME - ODOM_TIME))
DIFF=${DIFF#-} # absolute value

if [ "$DIFF" -gt 1 ]; then
    echo "[FAIL] Time drift between LiDAR and Odom is ${DIFF}s (> 1s)!"
else
    echo "[SUCCESS] Sensors are properly synchronized (Diff: ${DIFF}s)."
fi
echo "======================================"
```

---

### หมวดที่ 6: แนวทางการป้องกันปัญหาถาวร (Architectural Best Practices)

1. **ระบุค่า `use_sim_time` ให้ชัดเจนใน Launch Files:**
   - ในไฟล์ Launch ของ Simulation (เช่น `amv_sim.launch`) ให้ประกาศ:
   ```xml
   <param name="/use_sim_time" value="true" />
   ```
   - ในไฟล์ Bringup ของหุ่นยนต์จริง (เช่น `amv_base_connect.launch`) ให้บังคับปิดเสมอ:
   ```xml
   <param name="/use_sim_time" value="false" />
   ```
   เพื่อป้องกันไม่ให้ค่าตกค้างจากการเปิด Simulation ข้ามมายังหุ่นจริง

2. **ติดตั้ง Chrony Local NTP Master-Client ถาวร:**
   กำหนดให้หุ่นยนต์ `amv2` เป็น NTP Master Server ประจำวง Wi-Fi และให้โน้ตบุ๊ก `msi` ซิงก์เวลากับ `amv2` อัตโนมัติทุกวินาที ตัดปัญหาเรื่องการต้องพิมพ์ `sudo date -s` แบบ Manual

3. **การออกแบบ Remap ใน `robot_pose_ekf` ให้ครบถ้วน:**
   ตรวจเช็คชื่อ Topic ข้อมูล IMU เสมอ และใส่ `<remap from="imu_data" to="/imu/data" />` ให้ตรงกับไดรเวอร์ เพื่อไม่ให้ EKF รอจน Timeout (1 Hz)