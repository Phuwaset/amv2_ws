#!/usr/bin/env python3
import rospy
import rosnode
import math
import time
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
from amv_qr_detection.msg import QrDetectionStamped

class QrPoseLandmark():  

    tag_detect = 'None'
    qr_count = 0
    qr_px = 0
    qr_py = 0
    qr_distance = 100
    qr_detection_id = 'None'
    qr_set_position = 'None'
    qr_set_position_count = 0
    

    def tag_callback(self, tag_data):
        data_detection = tag_data.detections
        i = len(data_detection)
        if i > 0:
                tag_id = data_detection[0].id[0]
                self.qr_detection_id = '/tag' + str(tag_id)
        else:
                self.qr_detection_id = 'None'


        #loop for tag set position
        self.qr_count += 1
        if self.qr_count > 10:                    
            if i > 0:
                tag_id = data_detection[0].id[0]
                self.tag_detect = '/tag' + str(tag_id)
            else:
                self.tag_detect = 'None'
        else:
                self.tag_detect = 'None'  
    
    def qr_distance_callback(self, distance_data):
        self.qr_px = distance_data.pose.position.x
        self.qr_py = distance_data.pose.position.y
        self.qr_distance = math.sqrt(pow(self.qr_px,2) + pow(self.qr_py,2))
        #print (self.qr_distance)
        

    def __init__(self):
        rospy.init_node('qr_pose_landmark', anonymous=False)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.tag_callback)
        rospy.Subscriber('qr_distance', PoseStamped, self.qr_distance_callback)  
        pose_tf_pub = rospy.Publisher('/amv_pose_with_qr_landmark', PoseStamped, queue_size=1)
        reset_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        qr_detection_pub = rospy.Publisher('qr_detection', QrDetectionStamped, queue_size=1)
        self.parent_frame = rospy.get_param('parent_frame', default='/qr_landmark')
        self.child_frame = rospy.get_param('child_frame', default='/base_footprint')
        
        tf = TransformListener()

        time.sleep(5)

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():

            node_list = rosnode.get_node_names()
            node_checker1 = '/amcl' in node_list
            node_checker2 = '/move_base' in node_list
            node_checker3 = '/qr_pose_landmark' in node_list
            node_checker4 = '/map_server' in node_list

            try:
                
                if (self.tag_detect != 'None' and self.qr_distance < 1.2 and node_checker1 and node_checker2 and node_checker3 and node_checker4):
                    
                    #check tf between qr_landmark with tag
                    (trans_tag,rot_tag) = tf.lookupTransform(self.parent_frame, self.tag_detect, rospy.Time(0))          
                    
                    (trans,rot) = tf.lookupTransform(self.parent_frame, self.child_frame, rospy.Time(0))

                    amv_pose_with_qr = PoseStamped()
                    amv_pose_with_qr.header.frame_id = self.parent_frame
                    amv_pose_with_qr.pose.position.x = trans[0]
                    amv_pose_with_qr.pose.position.y = trans[1]
                    amv_pose_with_qr.pose.position.z = trans[2]
                    amv_pose_with_qr.pose.orientation.x = rot[0]
                    amv_pose_with_qr.pose.orientation.y = rot[1]
                    amv_pose_with_qr.pose.orientation.z = rot[2]
                    amv_pose_with_qr.pose.orientation.w = rot[3]
                    pose_tf_pub.publish(amv_pose_with_qr)

                    reset_position = PoseWithCovarianceStamped()
                    reset_position.header.frame_id = 'map'
                    reset_position.pose.pose.position.x = trans[0]
                    reset_position.pose.pose.position.y = trans[1]
                    reset_position.pose.pose.position.z = 0
                    reset_position.pose.pose.orientation.x = 0
                    reset_position.pose.pose.orientation.y = 0
                    reset_position.pose.pose.orientation.z = rot[2]
                    reset_position.pose.pose.orientation.w = rot[3]               
                    reset_pose_pub.publish(reset_position)
                    rospy.loginfo('amv position is set by qr detection landmark --> ' + self.tag_detect)

                    self.qr_set_position = self.tag_detect
                    self.tag_detect = 'None'
                    #self.qr_distance = 100
                    self.qr_count = 0
                    self.qr_set_position_count = 0
                else:
                    self.qr_set_position_count += 1
                    if self.qr_set_position_count > 5:
                        self.qr_set_position = 'None'

             
            except:
                rospy.logwarn('Cannot fint tf between qr_landmark to base_footprint')
            
            qr_detection = QrDetectionStamped()
            qr_detection.header.frame_id = self.parent_frame
            qr_detection.qr_data.qr_detection_id = self.qr_detection_id
            qr_detection.qr_data.qr_position_x = self.qr_px
            qr_detection.qr_data.qr_position_y = self.qr_py
            qr_detection.qr_data.qr_distance = self.qr_distance
            qr_detection.qr_data.qr_set_position = self.qr_set_position
            qr_detection_pub.publish(qr_detection)




            rate.sleep()    

if __name__ == '__main__':
    QrPoseLandmark()