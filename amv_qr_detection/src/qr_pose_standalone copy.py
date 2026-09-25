#!/usr/bin/env python3
import numpy as np
import rospy
import csv
import math
import time
import tf
from tf import TransformListener
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped
from apriltag_ros.msg import AprilTagDetectionArray
from amv_qr_detection.msg import LandmarkData

class QrPoseStandalone():    
    
    landmark_list = [] 
    tag_detect = 'None'

    def read_landmark_csv(self, path):
        print ('path', path)
        with open(path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            print ('landmark is readed')
            for row in reader:                
                if row[0] != 'tag_id':    #check header name --> tag_id,x,y,qz
                    print (row[0],row[1],row[2],row[3])  
                    landmark = LandmarkData()
                    landmark.tag_id = row[0]                    
                    landmark.x = float(row[1])
                    landmark.y = float(row[2])
                    landmark.qz = float(row[3])
                    self.landmark_list.append(landmark) 

    def tag_callback(self, tag_data):
        data_detection = tag_data.detections
        i = len(data_detection)        
        if i > 0:
            tag_id = data_detection[0].id[0]
            self.tag_detect = '/tag' + str(tag_id)
        else:
            self.tag_detect = 'None'
    
    def qr_to_map_publish(self):        
        (trans,rot) = self.tf_qr_to_map.lookupTransform(self.parent_frame, self.tag_detect, rospy.Time(0))
        euler_rot = euler_from_quaternion(rot,axes='sxyz')
        qr_pose_tf = PoseStamped()
        qr_pose_tf.header.frame_id = self.parent_frame + ' --> ' + self.tag_detect
        qr_pose_tf.pose.position.x = trans[0]
        qr_pose_tf.pose.position.y = trans[1]
        qr_pose_tf.pose.position.z = trans[2]
        qr_pose_tf.pose.orientation.x = 0
        qr_pose_tf.pose.orientation.y = 0
        qr_pose_tf.pose.orientation.z = euler_rot[2]
        qr_pose_tf.pose.orientation.w = 0
        self.qr_to_map_pub.publish(qr_pose_tf)
    
    def qr_to_base_publish(self):        
        (trans,rot) = self.tf_qr_to_base.lookupTransform(self.base_frame, self.tag_detect, rospy.Time(0))
        euler_rot = euler_from_quaternion(rot,axes='sxyz')
        qr_pose_tf = PoseStamped()
        qr_pose_tf.header.frame_id = self.base_frame + ' --> ' + self.tag_detect
        qr_pose_tf.pose.position.x = trans[0]
        qr_pose_tf.pose.position.y = trans[1]
        qr_pose_tf.pose.position.z = trans[2]
        qr_pose_tf.pose.orientation.x = 0
        qr_pose_tf.pose.orientation.y = 0
        qr_pose_tf.pose.orientation.z = euler_rot[2]
        qr_pose_tf.pose.orientation.w = 0
        self.qr_to_base_pub.publish(qr_pose_tf)
    
    def qr_distance_publish(self):
        (trans,rot) = self.tf_qr_distance.lookupTransform(self.camera_frame, self.tag_detect, rospy.Time(0))
        euler_rot = euler_from_quaternion(rot,axes='sxyz')
        qr_dist_tf = PoseStamped()
        qr_dist_tf.header.frame_id = self.camera_frame + ' --> ' + self.tag_detect
        qr_dist_tf.pose.position.x = trans[0]
        qr_dist_tf.pose.position.y = trans[1]
        qr_dist_tf.pose.position.z = trans[2]
        qr_dist_tf.pose.orientation.x = 0
        qr_dist_tf.pose.orientation.y = 0
        qr_dist_tf.pose.orientation.z = euler_rot[2]
        qr_dist_tf.pose.orientation.w = 0
        self.qr_distacne_pub.publish(qr_dist_tf)

    def amv_pose_calculate(self):
        for i in range(0, len(self.landmark_list)):
            if self.tag_detect == '/' + self.landmark_list[i].tag_id: 
                px_landmark = self.landmark_list[i].x
                py_landmark = self.landmark_list[i].y               
                qz_landmark = self.landmark_list[i].qz

                (trans,rot) = self.tf_qr_to_base.lookupTransform(self.base_frame, self.tag_detect, rospy.Time(0))
                euler_rot = euler_from_quaternion(rot,axes='sxyz')
                px_detect = trans[0]
                py_detect = trans[1]
                qz_detect = euler_rot[2]

                rotation_frame = qz_landmark - qz_detect 

                vector_landmark = np.array([[px_landmark],[py_landmark],[0]])
                vector_detect = np.array([[px_detect],[py_detect],[0]])
                vector_new = vector_landmark - vector_detect

                A = np.array([[vector_detect[0][0]],[vector_detect[1][0]],[vector_detect[2][0]],[1]])
                B = np.array([[math.cos(-qz_landmark), -math.sin(-qz_landmark), 0, 0],
                            [math.sin(-qz_landmark),  math.cos(-qz_landmark), 0, 0],
                            [0,                                 0,                  1, 0],
                            [0,                                 0,                  0, 1]])
        
                new_amv_pose = np.dot(B,A)
                print (rotation_frame)
                print (new_amv_pose) 

                q_new = tf.transformations.quaternion_from_euler(0, 0, rotation_frame)
                #print (q_new[3])

                reset_position = PoseWithCovarianceStamped()
                reset_position.header.frame_id = 'map'
                reset_position.pose.pose.position.x = new_amv_pose[0]
                reset_position.pose.pose.position.y = new_amv_pose[1]
                reset_position.pose.pose.position.z = 0
                reset_position.pose.pose.orientation.x = q_new[0]
                reset_position.pose.pose.orientation.y = q_new[1]
                reset_position.pose.pose.orientation.z = q_new[2]
                reset_position.pose.pose.orientation.w = q_new[3]             
                self.reset_pose_pub.publish(reset_position)             
                print('Set position with landmark --> ' + self.landmark_list[i].tag_id)
                break
            else:
                print('No landmark setting ' + self.landmark_list[i].tag_id)

    def __init__(self):
        rospy.init_node('qr_pose_standalone', anonymous=False)
        rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.tag_callback)

        self.reset_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1) 

        self.qr_to_map_pub = rospy.Publisher('/qr_to_map', PoseStamped, queue_size=1)
        self.qr_to_base_pub = rospy.Publisher('/qr_to_base', PoseStamped, queue_size=1)
        self.qr_distacne_pub = rospy.Publisher('/qr_distance', PoseStamped, queue_size=1)
        self.tf_qr_to_map = TransformListener()
        self.tf_qr_to_base = TransformListener()
        self.tf_qr_distance = TransformListener()

        self.parent_frame = rospy.get_param('parent_frame', default='/map')
        self.base_frame = rospy.get_param('base_frame', default='/base_footprint')    
        self.camera_frame = rospy.get_param('camera_frame', default='/camera_link')
        self.tag_frame = rospy.get_param('tag_frame', default='/tag0')        

        self.csv_path = rospy.get_param('~landmark_path', default='/home/minirw/amv_ws/src/amv_qr_detection/landmark/landmark.csv')
        self.read_landmark_csv(self.csv_path)        

        time.sleep(2)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():           
            #try:
                if (self.tag_detect != 'None'):
                    
                    #find distance and angle /map to /tag    
                    self.qr_to_map_publish()
                    #find distance and angle /base_footprint to /tag    
                    self.qr_to_base_publish() 
                    #find distance and angle /camera_link to /tag
                    self.qr_distance_publish() 

                    self.amv_pose_calculate()

                    self.tag_detect = 'None'

            #except:
                #rospy.logwarn('Cannot find tf between map to tag_dtections')

                rate.sleep()    

if __name__ == '__main__':
    QrPoseStandalone()