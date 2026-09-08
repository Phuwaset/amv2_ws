#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from tf.transformations import euler_from_quaternion
from tf import TransformListener
import math

class WaypointTeacher():
    
    def euclidian_dist(self,goal_point, current_pose):
        dx = goal_point.position.x - current_pose.position.x
        dy = goal_point.position.y - current_pose.position.y
        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

    def __init__(self):
        rospy.init_node('waypoint_teacher', anonymous=False)
        tf = TransformListener()
        rate = rospy.Rate(2)
        pose_tf_pub = rospy.Publisher("/initialpose_data", PoseWithCovarianceStamped, queue_size=1) 
        covarience = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942]
        last_pose = Pose()
        pub_pose = PoseWithCovarianceStamped()
        pub_pose.pose.covariance = covarience
        first_run = True
        while not rospy.is_shutdown(): 
            try:
                #print 'aaa'
                (trans,rot) = tf.lookupTransform('/map', '/laser_link', rospy.Time(0))
                #print 'aaa'
                current_pose_tf = Pose()
                current_pose_tf.position.x = trans[0]
                current_pose_tf.position.y = trans[1]
                current_pose_tf.position.z = trans[2]
                current_pose_tf.orientation.x = rot[0]
                current_pose_tf.orientation.y = rot[1]
                current_pose_tf.orientation.z = rot[2]
                current_pose_tf.orientation.w = rot[3]      
                #print 'aaa'         
                if first_run:
                    last_pose = current_pose_tf
                    first_run = False           
                dist = self.euclidian_dist(current_pose_tf,last_pose)
                #print 'aaa'
                if dist >= 1:
                    pub_pose.pose.pose = current_pose_tf
                    pose_tf_pub.publish(pub_pose)
                    last_pose = current_pose_tf
            except:
                rospy.logwarn('Failed to get current pose, did the "map" frame available? ,this might happen when run this script for the first time')
            rate.sleep()

if __name__ == '__main__':
    WaypointTeacher()