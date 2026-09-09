#!/bin/bash
# test_led_buzzer.sh - ทดสอบ LED/Buzzer AMV2 (9 ชุดหลัก)
# วิธีใช้: sudo systemctl stop amv-start.service
#         bash /home/minirw/amv_ws/src/amv_connect/debug_connect/test_led_buzzer.sh
#         sudo systemctl start amv-start.service

WS=/home/minirw/amv_ws
source "$WS/devel/setup.bash"

STARTED_MASTER=0
MASTER_PID=""
BASE_PID=""

cleanup() {
  echo ""
  echo ">>> กำลังปิดโหนดทดสอบ..."
  if [ -n "$BASE_PID" ] && kill -0 "$BASE_PID" 2>/dev/null; then
    kill "$BASE_PID" 2>/dev/null
    wait "$BASE_PID" 2>/dev/null
  fi
  if [ "$STARTED_MASTER" -eq 1 ]; then
    kill "$MASTER_PID" 2>/dev/null
  fi
  echo ">>> เสร็จสิ้น"
}
trap cleanup EXIT INT TERM

if ! rosparam list >/dev/null 2>&1; then
  echo ">>> เริ่ม roscore..."
  roscore > /tmp/amv_roscore.log 2>&1 &
  MASTER_PID=$!
  STARTED_MASTER=1
  for i in $(seq 1 30); do
    if rosparam list >/dev/null 2>&1; then
      break
    fi
    sleep 1
  done
fi

if rosnode list 2>/dev/null | grep -q "/amv_base"; then
  echo ">>> amv_base รันอยู่แล้ว (ข้ามการเปิด)"
else
  echo ">>> เริ่ม amv_base (สื่อสารบอร์ดควบคุม)..."
  rosrun amv_connect amv_base_node_28082026.py &
  BASE_PID=$!
fi

echo ">>> รอให้บอร์ดพร้อม..."
sleep 3

echo ">>> เริ่มทดสอบ LED/Buzzer (กด Ctrl+C เพื่อยกเลิก)"
rosrun amv_connect led_buzzer_test.py
