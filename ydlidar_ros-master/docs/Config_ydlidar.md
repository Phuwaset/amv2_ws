<!-- Write & Update 26/08/26 By Phuwaset Sibta {Engineer Team} -->
# ตรวจสอบ USB Path ของพอร์ตทั้งสองตัว

    ls -l /dev/serial/by-path/

    เห็นชื่อพอร์ตยาวๆ เช่น pci-0000:00:14.0-usb-0:6.2:1.0-port0 -> ../../ttyUSB3

# ตรวจสอบ KERNELS ของตัวหน้า (ttyUSB3)
    udevadm info -a -n /dev/ttyUSB3 | grep -m 1 "KERNELS=="

# ตรวจสอบ KERNELS ของตัวหลัง (ttyUSB2)
    udevadm info -a -n /dev/ttyUSB2 | grep -m 1 "KERNELS=="

# ปิดแก้ไขไฟล์ ydlidar.rules บนหุ่นยนต์ (amv2)

    sudo nano /etc/udev/rules.d/ydlidar.rules

# เปลี่ยนค่าในไฟล์เป็นชุดคำสั่งนี้ทั้งหมด

    # YDLIDAR Front (LiDAR หน้า)
    KERNEL=="ttyUSB*", KERNELS=="3-6.2", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ydlidar_front"

    # YDLIDAR Back (LiDAR หลัง)
    KERNEL=="ttyUSB*", KERNELS=="3-2.3", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="ydlidar_back"
    
# (กด Ctrl + O $\rightarrow$ Enter เพื่อบันทึก และ Ctrl + X เพื่อออก)
# บังคับให้อัปเดตและสร้าง Symlink ทันที

    sudo udevadm control --reload-rules
    sudo udevadm trigger --action=add --subsystem-match=tty

# ตรวจสอบพอร์ต
    
    ls -l /dev/ydlidar*
    
    Output -- should be same
        lrwxrwxrwx 1 root root 7 ... /dev/ydlidar_back -> ttyUSB2
        lrwxrwxrwx 1 root root 7 ... /dev/ydlidar_front -> ttyUSB3
