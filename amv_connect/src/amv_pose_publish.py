#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from tf import TransformListener
import math

class PoseCheck():
    is_pose_jumped = True

    def euclidian_dist(self,goal_point, current_pose):
        dx = goal_point.position.x - current_pose.position.x
        dy = goal_point.position.y - current_pose.position.y
        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))

    def __init__(self):
        rospy.init_node('robot_pose_streamer', anonymous=False)
        tf = TransformListener()
        rate = rospy.Rate(10)
        
        pose_str_pub = rospy.Publisher("/amv_pose_str", String, queue_size=1) 
        pose_tf_pub = rospy.Publisher("/amv_pose_tf", Pose, queue_size=1) 
        
        last_pose = Pose()        
        self.fail_count = 0
        
        while not rospy.is_shutdown(): 
            
            try:
                (trans,rot) = tf.lookupTransform('/map', '/base_link', rospy.Time(0))     #laser_link  change to base_link
                current_pose_str = 'X:'+str(trans[0])+',Y:'+str(trans[1])+',Z:'+str(trans[2])
                euler_rot = euler_from_quaternion(rot,axes='sxyz')
                
                ################################
                ## comment this line if you want euler
                #current_pose_str = current_pose_str + ',QX:'+str(rot[0])+',QY:'+str(rot[1])+',QZ:'+str(rot[2])+',QW:'+str(rot[3])

                ## comment this line if you want quatanion
                current_pose_str = current_pose_str + ',row:'+str(euler_rot[0])+',pitch:'+str(euler_rot[1])+',yaw:'+str(euler_rot[2])
                ################################
                current_pose_tf = Pose()
                current_pose_tf.position.x = trans[0]
                current_pose_tf.position.y = trans[1]
                current_pose_tf.position.z = trans[2]
                current_pose_tf.orientation.x = rot[0]
                current_pose_tf.orientation.y = rot[1]
                current_pose_tf.orientation.z = rot[2]
                current_pose_tf.orientation.w = rot[3]
                
                pose_str_pub.publish(current_pose_str)
                pose_tf_pub.publish(current_pose_tf)
                dist = self.euclidian_dist(current_pose_tf,last_pose)

                if dist > 0.5:
                    self.is_pose_jumped = True
                    rospy.logwarn('Robot pose jumped detected')
                last_pose = current_pose_tf
                self.fail_count = 0

            except:
                rospy.loginfo('Failed to get current pose, did the map frame available?')
            rate.sleep()

if __name__ == '__main__':
    PoseCheck()
