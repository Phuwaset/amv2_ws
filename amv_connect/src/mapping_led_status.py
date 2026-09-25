#!/usr/bin/env python3
import time
import rospy
from amv_connect.msg import LedCommandStamped


class MappingLedStatus:
    def __init__(self):
        rospy.init_node("mapping_led_status", anonymous=False)

        self.pub = rospy.Publisher("/led_command", LedCommandStamped, queue_size=1)

        # LED while mapping: green + fast flashing. buzzer via ~mapping_buzzer (default none)
        self.mapping_buzzer = rospy.get_param("~mapping_buzzer", "none")

        # wait briefly so /amv_base (subscriber) is connected before publishing
        t0 = rospy.Time.now()
        while self.pub.get_num_connections() == 0 and not rospy.is_shutdown() \
                and (rospy.Time.now() - t0).to_sec() < 3.0:
            rospy.sleep(0.1)

        self.publish_led("green", "fast", self.mapping_buzzer)
        rospy.loginfo("mapping_led_status: mapping mode -> green fast (buzzer=%s)",
                      self.mapping_buzzer)

        rospy.on_shutdown(self.on_shutdown)

    def publish_led(self, color, flashing, buzzer):
        msg = LedCommandStamped()
        msg.led_command.color = color
        msg.led_command.flashing = flashing
        msg.led_command.buzzer = buzzer
        self.pub.publish(msg)

    def on_shutdown(self):
        # mapping ended -> back to idle state of the robot: green solid, no sound
        rospy.loginfo("mapping_led_status: mapping ended -> idle (green solid)")
        for _ in range(5):
            self.publish_led("green", "none", "none")
            time.sleep(0.05)

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    MappingLedStatus().spin()
