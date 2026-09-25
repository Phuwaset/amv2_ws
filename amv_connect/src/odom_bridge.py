#!/usr/bin/python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdomBridge:
    def __init__(self):
        rospy.init_node("ekf_odom_bridge", anonymous=False)
        self.child = rospy.get_param('~child_frame_id', 'base_footprint')
        self.twist_source = rospy.get_param('~twist_source', '/odom_amv')
        self.twist = None
        rospy.Subscriber(self.twist_source, Odometry, self.on_twist, queue_size=1)
        rospy.Subscriber('/robot_pose_ekf/odom_combined',
                         PoseWithCovarianceStamped, self.on_pose, queue_size=1)
        self.pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def on_twist(self, msg):
        self.twist = msg.twist

    def on_pose(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'odom'
        odom.child_frame_id = self.child
        odom.pose = msg.pose
        if self.twist is not None:
            odom.twist = self.twist
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.angular.z = 0.0
        self.pub.publish(odom)

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    OdomBridge().run()
