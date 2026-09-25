#!/usr/bin/env python3
import threading
import time

import rospy
from amv_connect.msg import LedCommandStamped

TESTS = [
    {"name": "red_button",     "color": "orange",  "flashing": "fast",   "buzzer": "sound4", "desc": "กดปุ่มแดง: ไฟส้มกระพริบเร็ว + เสียง sound4"},
    {"name": "planning_ok",    "color": "green",   "flashing": "middle", "buzzer": "sound1", "desc": "วางแผนเส้นทางปกติ: ไฟเขียวกระพริบกลาง + เสียง sound1"},
    {"name": "planning_obs",   "color": "purple",  "flashing": "middle", "buzzer": "sound1", "desc": "วางแผน + เจอสิ่งกีดขวาง: ไฟม่วงกระพริบกลาง + เสียง sound1"},
    {"name": "recovery",       "color": "green",   "flashing": "fast",   "buzzer": "sound4", "desc": "Recovery path: ไฟเขียวกระพริบเร็ว + เสียง sound4"},
    {"name": "arrived",        "color": "blue",    "flashing": "middle", "buzzer": "none",   "desc": "ถึงสถานี: ไฟน้ำเงินกระพริบกลาง + เงียบ"},
    {"name": "error",          "color": "red",     "flashing": "middle", "buzzer": "sound4", "desc": "Error path: ไฟแดงกระพริบกลาง + เสียง sound4"},
    {"name": "idle_obstacle",  "color": "purple",  "flashing": "middle", "buzzer": "none",   "desc": "ว่าง + เจอสิ่งกีดขวาง: ไฟม่วงกระพริบกลาง + เงียบ"},
    {"name": "idle",           "color": "green",   "flashing": "none",   "buzzer": "none",   "desc": "ว่าง/ปกติ: ไฟเขียวติดนิ่ง + เงียบ"},
    {"name": "moving_out",     "color": "green",   "flashing": "fast",   "buzzer": "sound1", "desc": "ถอย/ออกจากสถานี: ไฟเขียวกระพริบเร็ว + เสียง sound1"},
]


class LedBuzzerTester:
    def __init__(self):
        rospy.init_node("led_buzzer_tester", anonymous=True)
        self.pub = rospy.Publisher("led_command", LedCommandStamped, queue_size=1)
        self.current = {"color": "none", "flashing": "none", "buzzer": "none"}
        self.stop_publish = False
        self.publish_thread = threading.Thread(target=self._publish_loop)
        self.publish_thread.daemon = True
        self.publish_thread.start()
        self.results = []

    def _make_msg(self, color, flashing, buzzer):
        msg = LedCommandStamped()
        msg.header.stamp = rospy.Time.now()
        msg.led_command.color = color
        msg.led_command.flashing = flashing
        msg.led_command.buzzer = buzzer
        return msg

    def set_command(self, color, flashing, buzzer):
        self.current = {"color": color, "flashing": flashing, "buzzer": buzzer}
        self.pub.publish(self._make_msg(color, flashing, buzzer))

    def _publish_loop(self):
        rate = rospy.Rate(20)
        while not self.stop_publish and not rospy.is_shutdown():
            self.pub.publish(self._make_msg(self.current["color"], self.current["flashing"], self.current["buzzer"]))
            rate.sleep()

    def ask(self, idx, test):
        prompt = "[{}/{}] {}: led={} flashing={} buzzer={} -> {} (y/n): ".format(
            idx, len(TESTS), test["name"], test["color"], test["flashing"], test["buzzer"], test["desc"])
        while True:
            answer = input(prompt).strip().lower()
            if answer in ("y", "yes"):
                return True
            if answer in ("n", "no"):
                return False
            print("กรุณาตอบ y/yes หรือ n/no")

    def run(self):
        time.sleep(1.0)
        if self.pub.get_num_connections() == 0:
            print("คำเตือน: ไม่มี subscriber บน led_command (amv_base ไม่รัน?) -> ไฟ/เสียงจะไม่แสดงผล")
        try:
            for i, test in enumerate(TESTS, 1):
                self.set_command(test["color"], test["flashing"], test["buzzer"])
                ok = self.ask(i, test)
                self.results.append((test, ok))
        except KeyboardInterrupt:
            print("\n>>> ยกเลิกโดยผู้ใช้")
        finally:
            self.set_command("none", "none", "none")
            self.stop_publish = True
            self.print_summary()

    def print_summary(self):
        print("\n=============== สรุปผลทดสอบ LED/Buzzer ===============")
        if not self.results:
            print("ยังไม่มีการทดสอบ")
            return
        passed = sum(1 for _, ok in self.results if ok)
        for test, ok in self.results:
            status = "PASS" if ok else "FAIL"
            print("[{}] {:<15} led={:<10} flashing={:<7} buzzer={:<7} -> {}".format(
                status, test["name"], test["color"], test["flashing"], test["buzzer"], test["desc"]))
        print("------------------------------------------------------")
        print("ผ่าน: {} / {}".format(passed, len(self.results)))
        print(">>> ไฟ/เสียงถูกรีเซ็ตเป็น off แล้ว")


if __name__ == "__main__":
    LedBuzzerTester().run()
