#!/usr/bin/env python3
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class SafetyZoneVisualizer:
    def __init__(self):
        rospy.init_node('safety_zone_visualizer', anonymous=False)

        # 1. โหลดพารามิเตอร์ขอบเขต Zone 1 (สอดคล้องกับ laser_safety.launch)
        self.frame_id = rospy.get_param('~frame_id', 'base_link')
        self.x_min = rospy.get_param('~x_min', 0.40)
        self.x_max = rospy.get_param('~x_max', 0.75)
        self.y_min = rospy.get_param('~y_min', -0.30)
        self.y_max = rospy.get_param('~y_max', 0.30)

        # 2. Publisher
        self.marker_pub = rospy.Publisher('/amv/safety_zone_marker', MarkerArray, queue_size=1, latch=True)
        self.rate = rospy.Rate(5)  # 5 Hz

    def generate_markers(self):
        marker_arr = MarkerArray()
        now = rospy.Time.now()

        # คำนวณมิติและจุดกึ่งกลางของกล่อง Zone 1
        center_x = (self.x_min + self.x_max) / 2.0
        center_y = (self.y_min + self.y_max) / 2.0
        size_x = abs(self.x_max - self.x_min)
        size_y = abs(self.y_max - self.y_min)
        size_z = 0.05  # ความหนาของแผ่นมาร์กเกอร์

        # --- 1. กล่องสี่เหลี่ยมสีแดงโปร่งแสง (CUBE) ---
        box = Marker()
        box.header.frame_id = self.frame_id
        box.header.stamp = now
        box.ns = "zone1_box"
        box.id = 1
        box.type = Marker.CUBE
        box.action = Marker.ADD
        box.pose.position.x = center_x
        box.pose.position.y = center_y
        box.pose.position.z = 0.05
        box.pose.orientation.w = 1.0
        box.scale.x = size_x
        box.scale.y = size_y
        box.scale.z = size_z
        box.color.r = 1.0
        box.color.g = 0.1
        box.color.b = 0.1
        box.color.a = 0.35  # ความโปร่งแสง
        marker_arr.markers.append(box)

        # --- 2. เส้นขอบสี่เหลี่ยมสีแดงทึบ (LINE_STRIP) ---
        lines = Marker()
        lines.header.frame_id = self.frame_id
        lines.header.stamp = now
        lines.ns = "zone1_border"
        lines.id = 2
        lines.type = Marker.LINE_STRIP
        lines.action = Marker.ADD
        lines.pose.orientation.w = 1.0
        lines.scale.x = 0.02  # ความหนาเส้นขอบ
        lines.color.r = 1.0
        lines.color.g = 0.0
        lines.color.b = 0.0
        lines.color.a = 0.95

        p1 = Point(x=self.x_min, y=self.y_min, z=0.08)
        p2 = Point(x=self.x_max, y=self.y_min, z=0.08)
        p3 = Point(x=self.x_max, y=self.y_max, z=0.08)
        p4 = Point(x=self.x_min, y=self.y_max, z=0.08)
        lines.points = [p1, p2, p3, p4, p1]
        marker_arr.markers.append(lines)

        # --- 3. ป้ายข้อความเตือน 3D (TEXT_VIEW_FACING) ---
        text = Marker()
        text.header.frame_id = self.frame_id
        text.header.stamp = now
        text.ns = "zone1_text"
        text.id = 3
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = center_x
        text.pose.position.y = center_y
        text.pose.position.z = 0.18
        text.pose.orientation.w = 1.0
        text.scale.z = 0.10
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = "ZONE 1: EMERGENCY STOP"
        marker_arr.markers.append(text)

        return marker_arr

    def run(self):
        while not rospy.is_shutdown():
            markers = self.generate_markers()
            self.marker_pub.publish(markers)
            self.rate.sleep()

if __name__ == '__main__':
    try:
        visualizer = SafetyZoneVisualizer()
        visualizer.run()
    except rospy.ROSInterruptException:
        pass