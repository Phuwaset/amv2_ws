#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import Twist


class JoyVelFilter:
    def __init__(self):
        rospy.init_node("joy_vel_filter", anonymous=False)

        self.filter_tau = rospy.get_param("~filter_tau", 0.15)              # s, low-pass time constant (0 = off)
        self.max_linear_accel = rospy.get_param("~max_linear_accel", 1.0)   # m/s^2
        self.max_angular_accel = rospy.get_param("~max_angular_accel", 2.0) # rad/s^2
        self.deadband_linear = rospy.get_param("~deadband_linear", 0.01)    # m/s
        self.deadband_angular = rospy.get_param("~deadband_angular", 0.02)  # rad/s
        self.input_timeout = rospy.get_param("~input_timeout", 1.5)         # s

        self.target_linear = 0.0
        self.target_angular = 0.0
        self.filtered_linear = 0.0
        self.filtered_angular = 0.0
        self.output_linear = 0.0
        self.output_angular = 0.0
        self.last_input_time = rospy.Time(0)
        self.active = False

        self.pub = rospy.Publisher("joy_vel", Twist, queue_size=1)
        rospy.Subscriber("joy_vel_raw", Twist, self.cmd_callback)

        self.rate = rospy.Rate(20)
        self.last_loop_time = rospy.Time.now()

        rospy.loginfo("joy_vel_filter started (tau=%.2f, accel=%.2f/%.2f, deadband=%.3f/%.3f, timeout=%.1f)",
                      self.filter_tau, self.max_linear_accel, self.max_angular_accel,
                      self.deadband_linear, self.deadband_angular, self.input_timeout)

        self.run()

    def cmd_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        if abs(linear) < self.deadband_linear:
            linear = 0.0
        if abs(angular) < self.deadband_angular:
            angular = 0.0

        self.target_linear = linear
        self.target_angular = angular
        self.last_input_time = rospy.Time.now()
        self.active = True

    @staticmethod
    def slew_limit(current, target, max_delta):
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def run(self):
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            dt = (now - self.last_loop_time).to_sec()
            self.last_loop_time = now
            if dt <= 0.0:
                dt = 0.05

            if self.active and (now - self.last_input_time).to_sec() > self.input_timeout:
                self.active = False

            if self.active:
                if self.filter_tau > 0.0:
                    alpha = 1.0 - math.exp(-dt / self.filter_tau)
                    self.filtered_linear += (self.target_linear - self.filtered_linear) * alpha
                    self.filtered_angular += (self.target_angular - self.filtered_angular) * alpha
                else:
                    self.filtered_linear = self.target_linear
                    self.filtered_angular = self.target_angular
            else:
                self.filtered_linear = 0.0
                self.filtered_angular = 0.0

            self.output_linear = self.slew_limit(self.output_linear, self.filtered_linear, self.max_linear_accel * dt)
            self.output_angular = self.slew_limit(self.output_angular, self.filtered_angular, self.max_angular_accel * dt)

            moving = abs(self.output_linear) > 1e-4 or abs(self.output_angular) > 1e-4
            if self.active or moving:
                cmd = Twist()
                cmd.linear.x = self.output_linear
                cmd.angular.z = self.output_angular
                self.pub.publish(cmd)

            self.rate.sleep()


if __name__ == "__main__":
    JoyVelFilter()
