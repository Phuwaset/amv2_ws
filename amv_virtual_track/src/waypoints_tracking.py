#!/usr/bin/env python3
import rospy
import math
import csv
import time
import actionlib
import tf
from tf import TransformListener
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from amv_connect.msg import BaseStatusStamped

class WayPointTracking():
    waypoints = []
    waypoints_name = []
    waypoints_mode = []
    waypoints_timer = []
    waypoints_pin = []     
    waypoints_count = 0
    waypoints_index = 0
    current_waypoint_index = -1

    def read_waypoint_csv(self, path):
        print ('path', path)
        with open(path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            print ('waypoint parameter readed')
            for row in reader:
                
                if row[0] != 'name':    #check header name --> name,mode,timer,pin,x,y,z,qx,qy,qz,qw
                    print (row[0])
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

    def euclidian_dist(self,goal_point, current_pose):
        dx = goal_point.position.x - current_pose.position.x
        dy = goal_point.position.y - current_pose.position.y
        return [math.sqrt(math.pow(dx, 2) + math.pow(dy, 2)), dx, dy]
    
    def find_direction(self,target):
        error = self.euclidian_dist(target, self.current_pose_tf)
        tar_rot = []
        tar_rot.append(target.orientation.x)
        tar_rot.append(target.orientation.y)
        tar_rot.append(target.orientation.z)
        tar_rot.append(target.orientation.w)
        goal_heading = euler_from_quaternion(tar_rot)
        goal_heading_yaw = goal_heading[2] * 180/math.pi
        if goal_heading_yaw < 0:
            goal_heading_yaw = 360 + goal_heading_yaw

        goal_direction = math.atan2(error[2], error[1]) * 180/math.pi
        if goal_direction < 0:
            goal_direction = 360 + goal_direction

        goal_heading_yaw_cal = goal_heading_yaw
        if abs(goal_heading_yaw_cal - self.current_heading_yaw) > 225:
            if goal_heading_yaw_cal > self.current_heading_yaw:
                self.current_heading_yaw = self.current_heading_yaw + 360
            else:
                goal_heading_yaw_cal = goal_heading_yaw_cal + 360
        

        if abs(goal_direction - self.current_heading_yaw) > 225:
            if goal_direction > self.current_heading_yaw:
                self.current_heading_yaw = self.current_heading_yaw + 360
            else:
                goal_direction = goal_direction + 360
        
        error_heading = goal_direction - self.current_heading_yaw
        error_heading_targer_cur = goal_heading_yaw_cal - self.current_heading_yaw
        return error_heading, error[0], error_heading_targer_cur                 

    def follow_waypoint(self):
        pose_dist_list = []
        pose_heading_list = []
        pose_ind_list = []
        pose_error_list = []

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('Connecting to move_base')
        self.client.wait_for_server()
        rospy.loginfo('Connected to move_base')
        rospy.loginfo('Starting a tf listner')
        self.tf = TransformListener()
        self.listener = tf.TransformListener()
        self.distance_tolerance = rospy.get_param('waypoint_distance_tolerance', 0.0)

        print ('No of Waypoints : ', len(self.waypoints))

        for i in range(0,len(self.waypoints)):
            error = self.find_direction(self.waypoints[i])

            #check heading of goal is not over -90 to 90 degree
            if(abs(error[0]) < 90) and (abs(error[2]) < 90):
                pose_heading_list.append(error[0])
                pose_dist_list.append(error[1])                
                pose_error_list.append(error[2])
                pose_ind_list.append(i)

            #rospy.loginfo("heading_err, distance_err, point_err: {0} {1} {2}".format(error[0], error[1], error[2]))
            print ('-----------------------------------------------')
            print ('Go to waypoint: ', i+1)  
            print (self.waypoints[i])

            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = self.frame_id
            goal.target_pose.pose.position = self.waypoints[i].position
            goal.target_pose.pose.orientation = self.waypoints[i].orientation
            rospy.loginfo('Executing move_base goal to position (x,y): %s, %s' %(self.waypoints[i].position.x, self.waypoints[i].position.y))
            rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
            print ('-----------------------------------------------')

            self.client.send_goal(goal)
            if not self.distance_tolerance > 0.0:

                self.client.wait_for_result()   #waiting until amv reach goal

                if (self.waypoints_mode[i] == 'A'):             
                    rospy.loginfo("Waiting for delay %d second" % self.waypoints_timer[i])
                    time.sleep(self.waypoints_timer[i])
                elif (self.waypoints_mode[i] == 'M'):
                    rospy.loginfo('Press green button to continue')   
                    while(self.green_button == False):                        
                        if (self.green_button): break
                else:
                    rospy.loginfo('Continue to next waypoint')
            else:
                #This is the loop which exist when the robot is near a certain goal point.
                distance = 10
                while(distance > self.distance_tolerance):
                    now = rospy.Time.now()
                    self.listener.waitForTransform(self.odom_frame_id, self.base_frame_id, now, rospy.Duration(4.0))
                    trans,rot = self.listener.lookupTransform(self.odom_frame_id,self.base_frame_id, now)
                    distance = math.sqrt(pow(self.waypoints[i].position.x-trans[0],2)+pow(self.waypoints[i].position.y-trans[1],2))
       
    def amv_pose_callback(self,data):
        self.current_pose_tf = data
        self.current_pose_x = data.position.x
        self.current_pose_y = data.position.y
        rot = []
        rot.append(self.current_pose_tf.orientation.x)
        rot.append(self.current_pose_tf.orientation.y)
        rot.append(self.current_pose_tf.orientation.z)
        rot.append(self.current_pose_tf.orientation.w)
     
        current_heading = euler_from_quaternion(rot)
        self.current_heading_yaw = current_heading[2] * 180/math.pi
        if self.current_heading_yaw < 0:
                self.current_heading_yaw = 360 + self.current_heading_yaw
    
    def amv_status_callback(self,data):
        self.green_button = data.base_status.green_button

    def __init__(self):
        rospy.init_node('waypoints_tracking', anonymous=False)

        self.frame_id = rospy.get_param('~goal_frame_id','map')
        self.odom_frame_id = rospy.get_param('~odom_frame_id','odom')
        self.base_frame_id = rospy.get_param('~base_frame_id','base_footprint')

        self.current_pose_tf = Pose()
        self.current_pose_x = 0
        self.current_pose_y = 0
        self.current_heading_yaw = 0

        self.green_button = False


        self.amv_pose_sub = rospy.Subscriber("amv_pose_tf", Pose, self.amv_pose_callback)
        self.amv_status_sub = rospy.Subscriber("amv_base_status", BaseStatusStamped, self.amv_status_callback)

        self.csv_path = rospy.get_param('~csv_path', default='/home/minirw/amv_ws/src/amv_navigation/waypoints/waypoints_list.csv')
        self.read_waypoint_csv(self.csv_path) 
        
        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown():
            self.follow_waypoint()

            rate.sleep()
        
if __name__ == '__main__':            
    WayPointTracking()
    

