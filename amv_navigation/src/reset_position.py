#!/usr/bin/env python3
import rospy
import math
import csv
import time
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped
from amv_connect.msg import BaseStatusStamped

class ResetPosition():
    waypoints = []
    waypoints_name = []
    waypoints_mode = []
    waypoints_timer = []
    waypoints_pin = []     
    waypoints_count = 0
    waypoints_index = 0
    current_waypoint_index = -1

    def read_waypoint_csv(self, path):
        #print ('path', path)
        with open(path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            #print ('waypoint parameter readed')
            for row in reader:
                
                if row[0] != 'name':    #check header name --> no,mode,timer,pin,x,y,z,qx,qy,qz,qw
                    #print (row[0])
                    self.waypoints_name.append(row[0])
                    self.waypoints_mode.append(row[1])
                    self.waypoints_timer.append(int(row[2]))
                    self.waypoints_pin.append(row[3])

                    pose = Pose()
                    pose.position.x = float(row[4])
                    pose.position.y = float(row[5])
                    pose.position.z = float(row[6])
                    pose.orientation.x = float(row[7])
                    pose.orientation.y = float(row[8])
                    pose.orientation.z = float(row[9])
                    pose.orientation.w = float(row[10])
                    self.waypoints.append(pose)  

    def check_current_station(self, current_pose):
        for i in range(0,len(self.waypoints)):
            dx = abs(self.waypoints[i].position.x - current_pose.position.x)
            dy = abs(self.waypoints[i].position.y - current_pose.position.y)
            dtheta = abs(self.waypoints[i].orientation.z - current_pose.orientation.z)            
            if dx <0.20 and dy < 0.20 and dtheta < 0.20:
                self.station_name = self.waypoints_name[i] 
                #print(self.station_name)
                break;
            else:
                self.station_name = "Out of station"

        self.current_station_pub.publish(self.station_name)       

    def reset_position(self, station_no):
        reset_position = PoseWithCovarianceStamped()
        reset_position.pose.pose.position.x = self.waypoints[station_no].position.x
        reset_position.pose.pose.position.y = self.waypoints[station_no].position.y
        reset_position.pose.pose.position.z = self.waypoints[station_no].position.z
        reset_position.pose.pose.orientation.x = self.waypoints[station_no].orientation.x
        reset_position.pose.pose.orientation.y = self.waypoints[station_no].orientation.y
        reset_position.pose.pose.orientation.z = self.waypoints[station_no].orientation.z
        reset_position.pose.pose.orientation.w = self.waypoints[station_no].orientation.w

        self.home_pose_pub.publish(reset_position)
        print ('amv position is set!')
        
    def amv_pose_callback(self,data):
        self.current_pose_tf = data
        rot = []
        rot.append(self.current_pose_tf.orientation.x)
        rot.append(self.current_pose_tf.orientation.y)
        rot.append(self.current_pose_tf.orientation.z)
        rot.append(self.current_pose_tf.orientation.w)

        self.check_current_station(self.current_pose_tf)
    
    def amv_status_callback(self,data):
        self.green_button = data.base_status.green_button
        self.green_button = data.base_status.red_button
        if self.green_button == True:
            self.reset_position(0) 
    
    def reset_station_callback(self,data):
        self.reset_station_no = int(data.data)
        self.reset_position(self.reset_station_no)  

    def __init__(self):
        rospy.init_node('reset_amv_position', anonymous=False)

        self.green_button = False
        self.red_button = False
        self.reset_station_no = 0
        self.station_name = "Out of station"

        self.amv_pose_sub = rospy.Subscriber("amv_pose_tf", Pose, self.amv_pose_callback)
        self.amv_status_sub = rospy.Subscriber("amv_base_status", BaseStatusStamped, self.amv_status_callback)
        self.amv_waypoint_sub = rospy.Subscriber("reset_station_no", String, self.reset_station_callback)
        
        self.current_station_pub = rospy.Publisher('amv_in_station', String, queue_size=1)
        self.home_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)

        self.csv_path = rospy.get_param('~csv_path', default='/home/minirw/amv_ws/src/amv_navigation/waypoints/waypoints_list.csv')
        self.read_waypoint_csv(self.csv_path) 
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            #time.sleep(0.001)
            rate.sleep()
        
if __name__ == '__main__':            
    ResetPosition()