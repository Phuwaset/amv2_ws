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

class RunLaunch():

    def start_node_direct():
        """
        Does work as well from service/topic callbacks directly using rosrun
        """    
        package = 'YOUR_PACKAGE'
        node_name = 'YOUR_NODE.py'

        command = "rosrun {0} {1}".format(package, node_name)

        p = subprocess.Popen(command, shell=True)

        state = p.poll()
        if state is None:
            rospy.loginfo("process is running fine")
        elif state < 0:
            rospy.loginfo("Process terminated with error")
        elif state > 0:
            rospy.loginfo("Process terminated without error")

    def start_node3():
        """
        Does not work if called from service/topic callbacks due to main signaling issue
        """
        package = 'amv_connect'
        launch_file = 'amv_base_connect.launch'
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch_file = os.path.join(rospkg.RosPack().get_path(package), 'launch', launch_file)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [launch_file])
        launch.start()

    def start_node4():
        """
        Does not work if called from service/topic callbacks due to main signaling issue
        """
        rospy.init_node('en_Mapping', anonymous=True)
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/minirw/amv_ws/src/amv_navigation/launch/waypoints_service.launch"])
        launch.start()
        rospy.loginfo("started")

        rospy.sleep(3)
        # 3 seconds later
        launch.shutdown()

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
        if self.navigation_flag == False:
            time.sleep(1)
            self.start_navigation()
            self.navigation_flag = True


    def __init__(self):
        rospy.init_node('run_launch', anonymous=True)

        self.amv_base_flag = False
        self.navigation_flag = False

        service = rospy.Service('start_amv_base', Trigger, self.start_amv_base_callback)
        service = rospy.Service('start_navigation', Trigger, self.start_navigation_callback)
        service = rospy.Service('start_amv_service', Trigger, self.start_amv_service_callback)
        service = rospy.Service('start_qr_detection', Trigger, self.start_qr_detection_callback)

        rospy.Subscriber("odom", PoseWithCovarianceStamped, self.odom_callback)

        time.sleep(1)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():

            node_list = rosnode.get_node_names()
            base_node = '/amv_base' in node_list

            if base_node == False and self.amv_base_flag == False:
                self.start_amv_base()
                self.amv_base_flag = True

            rate.sleep() 

if __name__ == '__main__':
    RunLaunch()