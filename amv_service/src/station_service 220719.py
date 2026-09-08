#!/usr/bin/env python3
from xml.sax.xmlreader import InputSource
import rospy
import math
import csv
import time
import actionlib
from geometry_msgs.msg import Pose, Point, PoseWithCovarianceStamped, Twist
from visualization_msgs.msg import Marker, MarkerArray
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool, String
from amv_connect.msg import BaseStatusStamped, LedCommandStamped
from amv_service.msg import StationData, NavigationStatus
from actionlib_msgs.msg import GoalStatusArray
from amv_service.srv import ReadStation, WriteStation, ResetPosition, GoStation

class StationService():  
    station_list = [] 
    green_button = False
    red_button = False
    pin_status = 'None'
    pin_command = 'None'
    pin_counter_check = 0
    current_station = 'Out of station'
    reset_station = 'None'
    going_station = 'None'
    navigation_status = 'None'
    obstacle = 'None'

    led_color ='none'
    led_flashing = 'none'
    buzzer = 'none'

    def read_station_csv(self, path):
        print ('path', path)
        with open(path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            print ('station service is readed')
            for row in reader:                
                if row[0] != 'name':    #check header name --> no,mode,timer,pin,x,y,z,qx,qy,qz,qw
                    print (row[0])  
                    station = StationData()
                    station.name = row[0]
                    station.mode = row[1]
                    station.timer = int(row[2])
                    station.pin = row[3]
                    station.x = float(row[4])
                    station.y = float(row[5])
                    station.z = float(row[6])
                    station.qx = float(row[7])
                    station.qy = float(row[8])
                    station.qz = float(row[9])
                    station.qw = float(row[10])
                    station.theta = float(row[11])
                    self.station_list.append(station)        
 
    def read_station_service(self, req):
        ret = ReadStation._response_class()
        status = ''
        no_of_station = 0
        _station_data = []   

        if req.read_action:
            if len(self.station_list) > 0:
                status = 'Read complete ' + str(len(self.station_list)) + ' stations'
                no_of_station = len(self.station_list)
                _station_data = self.station_list
            else:
                status = 'No station list'
                no_of_station = 0
        else:
            status = 'Read action flag is false'
        
        print(status)
        ret.status = status
        ret.station_total = no_of_station
        ret.station_data = _station_data
        return ret
    
    def reset_position_service(self, req):
        ret = ResetPosition._response_class()
        status = ''
        target_reset_position = req.target_station             
        
        for i in range(0, len(self.station_list)):
            if target_reset_position == self.station_list[i].name:                
                reset_position = PoseWithCovarianceStamped()
                reset_position.header.frame_id = 'map'
                reset_position.pose.pose.position.x = self.station_list[i].x
                reset_position.pose.pose.position.y = self.station_list[i].y
                reset_position.pose.pose.position.z = self.station_list[i].z
                reset_position.pose.pose.orientation.x = self.station_list[i].qx
                reset_position.pose.pose.orientation.y = self.station_list[i].qy
                reset_position.pose.pose.orientation.z = self.station_list[i].qz
                reset_position.pose.pose.orientation.w = self.station_list[i].qw                
                self.reset_pose_pub.publish(reset_position)
                status = 'Reset position complete at station --> ' + self.station_list[i].name

                self.reset_station = self.station_list[i].name

                break
            else:
                status = 'No reset station --> ' + target_reset_position

        print (status)
        ret.status = status
        return ret
    
    def go_station_service(self, req):
        ret = GoStation._response_class()
        status = ''
        target_go_station = req.target_station      
        
        for i in range(0, len(self.station_list)):
            if target_go_station == self.station_list[i].name:
                self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
                rospy.loginfo('Connecting to move_base')
                self.client.wait_for_server()
                rospy.loginfo('Connected to move_base')

                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = 'map'
                goal.target_pose.pose.position.x = self.station_list[i].x
                goal.target_pose.pose.position.y = self.station_list[i].y
                goal.target_pose.pose.position.z = self.station_list[i].z
                goal.target_pose.pose.orientation.x = self.station_list[i].qx
                goal.target_pose.pose.orientation.y = self.station_list[i].qy
                goal.target_pose.pose.orientation.z = self.station_list[i].qz
                goal.target_pose.pose.orientation.w = self.station_list[i].qw
                self.client.send_goal(goal)

                print ('------------------------------------------------------------------------------')
                rospy.loginfo('AMV start to station ' + self.station_list[i].name)
                rospy.loginfo("To cancel the goal: 'rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}'")
                print ('------------------------------------------------------------------------------')
                
                status = 'Going to station --> ' + self.station_list[i].name
                self.going_station = self.station_list[i].name
                self.pin_command = self.station_list[i].pin
                self.backward_complete = False #clear move backward
                self.forward_complete = False #clear move forward
                break
            else:
                status = 'No go station --> ' + target_go_station
                self.going_station = 'None'

        print (status)
        ret.status = status
        return ret
    
    def amv_status_callback(self,data):
        self.green_button = data.base_status.green_button
        self.red_button = data.base_status.red_button
        
        if self.red_button:  
            reset_home = rospy.ServiceProxy('reset_position', ResetPosition)
            result_reset_home = reset_home('Material Room')

        if data.base_status.linear_down_conf == True:
            self.pin_status = 'Down'
        if data.base_status.linear_up_conf == True:
            self.pin_status = 'Up'
    
    def safety_zone1_callback(self, data):
        if data.data == True:
            self.obstacle = 'Detecting'
        else:
            self.obstacle = 'None'

    def amv_pose_callback(self,data):
        self.current_pose = data        
        self.check_current_station(self.current_pose)
    
    def check_current_station(self, current_pose):
        for i in range(0,len(self.station_list)):
            dx = abs(self.station_list[i].x - current_pose.position.x)
            dy = abs(self.station_list[i].y - current_pose.position.y)
            dtheta = abs(self.station_list[i].qz - current_pose.orientation.z)            
            if dx <0.30 and dy < 0.30 and dtheta < 0.20:
                self.current_station = self.station_list[i].name
                break
            else:
                self.current_station = "Out of station"

    def movebase_status_callback(self, data):
        if self.red_button == True:
            self.led_color = 'orange'
            self.led_flashing = 'fast'
            self.buzzer = 'sound4'

        else:
            i = len(data.status_list)
            if i > 0:
                movebase_status = data.status_list[i-1].status
                if movebase_status == 1:
                    self.navigation_status = 'Got path planning'
                    self.pin_counter_check = 0 #clear pin_command
                    if self.obstacle == 'Detecting':
                        self.led_color = 'purple'
                    else:
                        self.led_color = 'green'

                    self.led_flashing = 'middle'
                    self.buzzer = 'sound1'

                elif movebase_status == 2:
                    self.navigation_status = 'Recovery path planning'
                    self.led_color = 'green'
                    self.led_flashing = 'fast'
                    self.buzzer = 'sound4'

                elif movebase_status == 3:
                    self.navigation_status = 'Arrived station'
                    self.going_station = 'Stop at station'
                    self.led_color = 'blue'
                    self.led_flashing = 'middle'
                    self.buzzer = 'none'

                    #Send pin lock command with arrived goal
                    if self.pin_counter_check < 15:
                        if self.pin_counter_check > 10:   #Delay before send pin command
                            self.pin_command_pub.publish(self.pin_command)
                            self.stop_pose = self.current_pose
                        self.pin_counter_check += 1

                    #Move backward from station
                    if self.pin_command == 'Down':
                        if self.pin_status == 'Down':
                            print('call backward function')
                            #self.moving_backward()
                            self.moving_forward()

                            if self.backward_complete == True:
                                go_next_station = rospy.ServiceProxy('go_station', GoStation)
                                result_go_next_station = go_next_station(self.next_station)
                                print ('go next station after backward')
                            
                            if self.forward_complete == True:
                                go_next_station = rospy.ServiceProxy('go_station', GoStation)
                                result_go_next_station = go_next_station(self.next_station)
                                print ('go next station after forward')


                elif movebase_status == 4:
                    self.navigation_status = 'Error path planning'
                    self.led_color = 'red'
                    self.led_flashing = 'middle'
                    self.buzzer = 'sound4'
            else:
                self.navigation_status = 'None'
                if self.obstacle == 'Detecting':
                    self.led_color = 'purple'
                    self.led_flashing = 'middle'
                    self.buzzer = 'none'
                else:
                    self.led_color = 'green'
                    self.led_flashing = 'none'
                    self.buzzer = 'none'
        
    def navigation_status_publish(self):
        _navigation_status = NavigationStatus()
        _navigation_status.current_station = self.current_station
        _navigation_status.reset_station = self.reset_station
        _navigation_status.going_station = self.going_station
        _navigation_status.navigation_status = self.navigation_status
        _navigation_status.pin_lock = self.pin_status
        _navigation_status.obstacle = self.obstacle

        self.amv_navigation_pub.publish(_navigation_status)
    
    def led_command_publish(self):
        data = LedCommandStamped()
        data.led_command.color = self.led_color
        data.led_command.flashing = self.led_flashing
        data.led_command.buzzer = self.buzzer

        self.led_command_pub.publish(data)

    def euclidian_dist(self, current, stop):
        dx = current.position.x - stop.position.x
        dy = current.position.y - stop.position.y
        return math.sqrt(math.pow(dx, 2) + math.pow(dy, 2))  
    
    def moving_backward(self):
        twist = Twist()
        dist = self.euclidian_dist(self.current_pose, self.stop_pose)
        if dist < 1.0:
            twist.linear.x = -0.1
            twist.angular.z = 0
            self.led_color = 'green'
            self.led_flashing = 'fast'
            self.buzzer = 'sound1'
            self.backward_complete = False            
            print(dist)
            print('amv is moving back from station')
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            self.backward_complete = True
        
        self.moveback_pub.publish(twist)

    def moving_forward(self):
        twist = Twist()
        dist = self.euclidian_dist(self.current_pose, self.stop_pose)
        if dist < 0.65:
            twist.linear.x = 0.1
            twist.angular.z = 0
            self.led_color = 'green'
            self.led_flashing = 'fast'
            self.buzzer = 'sound1'
            self.forward_complete = False            
            print(dist)
            print('amv is moving forward from station')
        else:
            twist.linear.x = 0
            twist.angular.z = 0
            self.forward_complete = True
        
        self.moveback_pub.publish(twist)


    def __init__(self):
        rospy.init_node('station_service', anonymous=False)

        self.current_pose =Pose()
        self.stop_pose =Pose()
        self.backward_complete = False
        self.forward_complete = False
        self.next_station = 'Line7'

        self.csv_path = rospy.get_param('~station_path', default='/home/minirw/amv_ws/src/amv_service/station/station_list.csv')
        self.read_station_csv(self.csv_path)   

        self.read_station_srv = rospy.Service('read_station', ReadStation, self.read_station_service)
        #self.write_station_srv = rospy.Service('write_station', WriteStation, self.write_station_service)
        self.reset_position_srv = rospy.Service('reset_position', ResetPosition, self.reset_position_service)
        self.go_station_srv = rospy.Service('go_station', GoStation, self.go_station_service)

        self.amv_pose_sub = rospy.Subscriber("amv_pose_tf", Pose, self.amv_pose_callback)
        self.amv_status_sub = rospy.Subscriber("amv_base_status", BaseStatusStamped, self.amv_status_callback)
        self.movebase_status_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self.movebase_status_callback)
        self.obstacle_sub = rospy.Subscriber("zone1", Bool, self.safety_zone1_callback)

        self.reset_pose_pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size=1)
        self.amv_navigation_pub = rospy.Publisher('amv_navigation_status', NavigationStatus, queue_size=1)
        self.pin_command_pub = rospy.Publisher('pin_command', String, queue_size=1)
        self.led_command_pub = rospy.Publisher('led_command', LedCommandStamped, queue_size=1)
        self.moveback_pub = rospy.Publisher("nav_vel", Twist, queue_size=1)

        rate = rospy.Rate(10)
        
        while not rospy.is_shutdown(): 
            self.navigation_status_publish()  
            self.led_command_publish()          
            rate.sleep()
        
if __name__ == '__main__':            
    StationService()
    

