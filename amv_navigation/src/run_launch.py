#!/usr/bin/env python3
import time
import os
import rospy
import rosnode
import rospkg
import subprocess
import roslaunch
from std_srvs.srv import Trigger, TriggerResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
from actionlib_msgs.msg import GoalStatusArray
from sensor_msgs.msg import Image

class RunLaunch():
    
    def start_amv_base(self):
        """
        Does work as well from service/topic callbacks using launch files
        """    
        package = 'amv_connect'
        launch_file = 'amv_base_connect.launch'

        command = "roslaunch  {0} {1}".format(package, launch_file)

        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("amv_base_connect process is running fine")
        elif state < 0:
            rospy.loginfo("amv_base_connect process terminated with error")
        elif state > 0:
            rospy.loginfo("amv_base_connect process terminated without error")

    def start_amv_base_callback(self, req):

        # edit below for other options
        self.start_amv_base()

        return TriggerResponse(success=True)

    def start_navigation(self):
        """
        Does work as well from service/topic callbacks using launch files
        """    
        package = 'amv_navigation'
        launch_file = 'navigation.launch'

        command = "roslaunch  {0} {1}".format(package, launch_file)

        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("amv_navigation process is running fine")
        elif state < 0:
            rospy.loginfo("amv_navigation process terminated with error")
        elif state > 0:
            rospy.loginfo("amv_navigation process terminated without error")

    def start_navigation_callback(self, req):

        # edit below for other options
        self.start_navigation()

        return TriggerResponse(success=True)

    def start_amv_service(self):
        """
        Does work as well from service/topic callbacks using launch files
        """    
        package = 'amv_service'
        launch_file = 'amv_control_service.launch'

        command = "roslaunch  {0} {1}".format(package, launch_file)

        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("amv_service process is running fine")
        elif state < 0:
            rospy.loginfo("amv_service process terminated with error")
        elif state > 0:
            rospy.loginfo("amv_service process terminated without error")

    def start_amv_service_callback(self, req):

        # edit below for other options
        self.start_amv_service()

        return TriggerResponse(success=True)


    def start_qr_detection(self):
        """
        Does work as well from service/topic callbacks using launch files
        """    
        package = 'amv_qr_detection'
        launch_file = 'qr_landmark.launch'

        command = "roslaunch  {0} {1}".format(package, launch_file)

        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("QR detection process is running fine")
        elif state < 0:
            rospy.loginfo("QR detection process terminated with error")
        elif state > 0:
            rospy.loginfo("QR detection process terminated without error")

    def start_qr_detection_callback(self, req):

        # edit below for other options
        self.start_qr_detection()

        return TriggerResponse(success=True)

    def odom_callback(self, data):

        if self.count_odom < 100:
            self.count_odom += 1

        if self.amv_navigation_flag == False and self.count_odom > 10:
            time.sleep(1)
            self.start_navigation()
            self.amv_navigation_flag = True
            self.count_odom = 0
        
    def amcl_callback(self, data):
       
        if self.amcl_flag == False:
            time.sleep(1)
            self.amcl_flag = True

    def movebase_callback(self, data):
        if self.amv_service_flag == False:
            time.sleep(3)
            self.start_amv_service()
            self.amv_service_flag = True
    
    def image_callback(self, data):        
        if self.qr_detection_flag == False and self.amv_service_flag == True and self.amcl_flag == True:
            time.sleep(5)
            self.start_qr_detection()
            self.qr_detection_flag = True
            print('--------------qr detection start----------------')

    def __init__(self):
        rospy.init_node('run_launch', anonymous=True)

        self.count_odom = 0
        self.count_amcl = 0
        self.amv_base_flag = False
        self.amcl_flag = False
        self.amv_navigation_flag = False
        self.amv_service_flag = False
        self.qr_detection_flag = False

        service = rospy.Service('start_amv_base', Trigger, self.start_amv_base_callback)
        service = rospy.Service('start_navigation', Trigger, self.start_navigation_callback)
        service = rospy.Service('start_amv_service', Trigger, self.start_amv_service_callback)
        service = rospy.Service('start_qr_detection', Trigger, self.start_qr_detection_callback)

        rospy.Subscriber("odom", PoseWithCovarianceStamped, self.odom_callback)
        rospy.Subscriber("amcl_pose", PoseWithCovarianceStamped, self.amcl_callback)
        rospy.Subscriber("move_base/status", GoalStatusArray, self.movebase_callback)
        rospy.Subscriber("camera/color/image_raw", Image, self.image_callback)

        time.sleep(1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            node_list = rosnode.get_node_names()
            base_node = '/amv_base' in node_list

            if base_node == False and self.amv_base_flag == False:
                self.start_amv_base()                
                self.amv_base_flag = True
            
            rate.sleep() 

if __name__ == '__main__':
    RunLaunch()