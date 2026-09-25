#!/usr/bin/env python3

import rospy
import math
#from math import pi
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String
from amv_statemachine.srv import GetWayPoint

class WayPointScheduler():
    waypoints = []
    stations = []
    waypoints_name = []
    stations_name = []
    stations_timer = []
    
    waypoints_count = 0
    waypoints_index = 0
    current_waypoint_index = -1

    def import_param_waypoint_csv(self, path):
        print ('path', path)
        with open(path, 'r') as param_file:
            params_list = param_file.readlines()

        if len(params_list) > 0:
            if params_list[0] == 'name,x,y,z,qx,qy,qz,qw\n':
                print ('waypoint parameter readed')
                for i in range(1, len(params_list)):
                    params = params_list[i].split(',')
                    self.waypoints_name.append(params[0])
                    pose = Pose()
                    pose.position.x = float(params[1])
                    pose.position.y = float(params[2])
                    pose.position.z = float(params[3])
                    pose.orientation.x = float(params[4])
                    pose.orientation.y = float(params[5])
                    pose.orientation.z = float(params[6])
                    pose.orientation.w = float(params[7])
                    self.waypoints.append(pose)
            else:
                print ('wrong files format')
        else:
            print ('file empty')

    def import_param_station_csv(self, path):
        print ('path', path)
        with open(path, 'r') as param_file:
            params_list = param_file.readlines()

        if len(params_list) > 0:
            if params_list[0] == 'name,x,y,z,qx,qy,qz,qw,waiting_time\n':
                print ('station parameter readed')
                for i in range(1, len(params_list)):
                    params = params_list[i].split(',')
                    self.stations_name.append(params[0])
                    pose = Pose()
                    pose.position.x = float(params[1])
                    pose.position.y = float(params[2])
                    pose.position.z = float(params[3])
                    pose.orientation.x = float(params[4])
                    pose.orientation.y = float(params[5])
                    pose.orientation.z = float(params[6])
                    pose.orientation.w = float(params[7])
                    self.stations.append(pose)
                    self.stations_timer.append(int(params[8]))
            else:
                print ('wrong files format')
        else:
            print ('file empty')

    def add_waypoint_name(self, pose, name):
        waypoint_marker_name = Marker()
        waypoint_marker_name.ns = 'waypoints_name'
        waypoint_marker_name.id = self.id
        self.id = self.id + 1
        waypoint_marker_name.type = Marker.TEXT_VIEW_FACING
        waypoint_marker_name.action = Marker.ADD
        waypoint_marker_name.lifetime = rospy.Duration(0)
        waypoint_marker_name.scale.z = 0.15
        waypoint_marker_name.color.r = 0
        waypoint_marker_name.color.g = 0
        waypoint_marker_name.color.b = 0
        waypoint_marker_name.color.a = 1
        waypoint_marker_name.header.frame_id = 'map'
        waypoint_marker_name.header.stamp = rospy.Time.now()
        waypoint_marker_name.pose = pose
        waypoint_marker_name.text = name
        return waypoint_marker_name
    
    def add_station_name(self, pose, name):
        waypoint_marker_name = Marker()
        waypoint_marker_name.ns = 'stations_name'
        waypoint_marker_name.id = self.id
        self.id = self.id + 1
        waypoint_marker_name.type = Marker.TEXT_VIEW_FACING
        waypoint_marker_name.action = Marker.ADD
        waypoint_marker_name.lifetime = rospy.Duration(0)
        waypoint_marker_name.scale.z = 0.15
        waypoint_marker_name.color.r = 0
        waypoint_marker_name.color.g = 0
        waypoint_marker_name.color.b = 0
        waypoint_marker_name.color.a = 1
        waypoint_marker_name.header.frame_id = 'map'
        waypoint_marker_name.header.stamp = rospy.Time.now()
        waypoint_marker_name.pose = pose
        waypoint_marker_name.text = name
        return waypoint_marker_name

    def init_waypoint_markers(self):
        
        # Set up our waypoint markers
        marker_scale = 0.05
        marker_lifetime = 0  # 0 is forever
        marker_ns_l = 'waypoints_line'
        marker_ns_c = 'waypoints_circle'
        marker_ns_n = 'waypoints_name'

        self.id = 0
        marker_id = 0
        marker_line_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}
        marker_circle_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
      
        # Define a marker publisher.
        self.marker_pub = rospy.Publisher(
            'waypoint_markers_p2p', MarkerArray, queue_size=5)

        self.waypoint_marker_array = MarkerArray()
       
        # Initialize the marker points list.
        self.waypoint_markers_line = Marker()
        self.waypoint_markers_line.ns = marker_ns_l
        self.waypoint_markers_line.id = marker_id
        self.waypoint_markers_line.type = Marker.LINE_LIST
        self.waypoint_markers_line.action = Marker.ADD
        self.waypoint_markers_line.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_markers_line.scale.x = 0.04 #Path size
        self.waypoint_markers_line.scale.y = 0.04
        self.waypoint_markers_line.color.r = marker_line_color['r']
        self.waypoint_markers_line.color.g = marker_line_color['g']
        self.waypoint_markers_line.color.b = marker_line_color['b']
        self.waypoint_markers_line.color.a = marker_line_color['a']

        self.waypoint_markers_line.header.frame_id = 'map'
        self.waypoint_markers_line.header.stamp = rospy.Time.now()
        self.waypoint_markers_line.points = list()

        self.waypoint_markers_circle = Marker()
        self.waypoint_markers_circle.ns = marker_ns_c
        self.waypoint_markers_circle.id = marker_id
        self.waypoint_markers_circle.type = Marker.SPHERE_LIST
        self.waypoint_markers_circle.action = Marker.ADD
        self.waypoint_markers_circle.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_markers_circle.scale.x = 0.1   #Waypoint circle size
        self.waypoint_markers_circle.scale.y = 0.1
        self.waypoint_markers_circle.scale.z = 0.1
        self.waypoint_markers_circle.color.r = marker_circle_color['r']
        self.waypoint_markers_circle.color.g = marker_circle_color['g']
        self.waypoint_markers_circle.color.b = marker_circle_color['b']
        self.waypoint_markers_circle.color.a = marker_circle_color['a']
        self.waypoint_markers_circle.header.frame_id = 'map'
        self.waypoint_markers_circle.header.stamp = rospy.Time.now()
        self.waypoint_markers_circle.points = list()

        self.waypoint_marker_name = Marker()
        self.waypoint_marker_name.ns = marker_ns_n
        self.waypoint_marker_name.id = marker_id
        self.waypoint_marker_name.type = Marker.TEXT_VIEW_FACING
        self.waypoint_marker_name.action = Marker.ADD
        self.waypoint_marker_name.lifetime = rospy.Duration(marker_lifetime)
        self.waypoint_marker_name.scale.z = 1
        self.waypoint_marker_name.color.r = marker_line_color['r']
        self.waypoint_marker_name.color.g = marker_line_color['g']
        self.waypoint_marker_name.color.b = marker_line_color['b']
        self.waypoint_marker_name.color.a = marker_line_color['a']
        self.waypoint_marker_name.header.frame_id = 'map'
        self.waypoint_marker_name.header.stamp = rospy.Time.now()
        self.waypoint_marker_name.points = list()
    
    def draw_arrow(self,pose):
        marker_arrow_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}
        #marker_scale = 0.05
        marker_lifetime = 0  # 0 is forever
        marker_ns_c = 'waypoints_arrow'
        #self.id = 0
        marker_id = 0

        waypoint_markers_arrow = Marker()
        waypoint_markers_arrow.ns = marker_ns_c
        waypoint_markers_arrow.id = marker_id
        waypoint_markers_arrow.type = Marker.ARROW
        waypoint_markers_arrow.action = Marker.ADD
        waypoint_markers_arrow.lifetime = rospy.Duration(marker_lifetime)
        waypoint_markers_arrow.scale.x = 0.5
        waypoint_markers_arrow.scale.y = 0.15
        waypoint_markers_arrow.scale.z = 0.01
        waypoint_markers_arrow.color.r = marker_arrow_color['r']
        waypoint_markers_arrow.color.g = marker_arrow_color['g']
        waypoint_markers_arrow.color.b = marker_arrow_color['b']
        waypoint_markers_arrow.color.a = marker_arrow_color['a']
        waypoint_markers_arrow.header.frame_id = 'map'
        waypoint_markers_arrow.header.stamp = rospy.Time.now()
        waypoint_markers_arrow.pose = pose
        return 

    def draw_markers(self):
        for i in range(0, len(self.waypoints_name)):
            waypoint_marker_name_new = self.add_waypoint_name(self.waypoints[i], self.waypoints_name[i])
            self.waypoint_marker_array.markers.append(waypoint_marker_name_new)

        for i in range(0, len(self.stations_name)):
            station_marker_name_new = self.add_station_name(self.stations[i], self.stations_name[i])
            self.waypoint_marker_array.markers.append(station_marker_name_new)

        for i in range(0, len(self.waypoints)-1):
            p1 = Point()
            p2 = Point()
            p1 = self.waypoints[i].position
            p2 = self.waypoints[i+1].position
            self.waypoint_markers_line.points.append(p1)
            self.waypoint_markers_line.points.append(p2)
            self.waypoint_markers_circle.points.append(p1)
            # waypoint_marker_name_new = self.add_name(
            # self.waypoints[i], self.waypoints_name[i])
            # self.waypoint_marker_array.markers.append(waypoint_marker_name_new)
        p1 = Point()
        p2 = Point()
        p1 = self.waypoints[len(self.waypoints)-1].position
        p2 = self.waypoints[0].position
        self.waypoint_markers_line.points.append(p1)
        self.waypoint_markers_line.points.append(p2)
        self.waypoint_markers_circle.points.append(p1)
        # waypoint_marker_name_new = self.add_name(
        # self.waypoints[len(self.waypoints)-1], self.waypoints_name[len(self.waypoints)-1])
        # self.waypoint_marker_array.markers.append(waypoint_marker_name_new)
        self.waypoint_marker_array.markers.append(self.waypoint_markers_line)
        self.waypoint_marker_array.markers.append(self.waypoint_markers_circle)
        #self.waypoint_marker_array.markers.append(self.waypoint_markers_arrow)

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
    
    def find_next_station_arr(self,reached_station):
        nd = self.stations_name.index(reached_station)
        try:
            rospy.logwarn("tried")

            ind = self.stations_name.index(reached_station)
          
            
            if ind == len(self.stations)-1:
                next_ind = 0
            else:
                next_ind = ind + 1
            rospy.logwarn("NEXT IND {0}".format(next_ind))

            return [self.stations[next_ind], self.stations_name[next_ind], self.stations_timer[next_ind]]
        except:
            return []

    def find_next_station(self,pose,reached_station):
        pose_dist_list = []
        pose_heading_list = []
        pose_ind_list = []
        pose_error_list = []
        for i in range(0,len(self.stations)):
            error = self.find_direction(self.stations[i])
            if(abs(error[0]) < 150) and (abs(error[1]) <= 80) and (abs(error[1]) >= 0.5):
                pose_dist_list.append(error[1])
                pose_heading_list.append(error[0])
                pose_error_list.append(error[2])
                pose_ind_list.append(i)
            #print 'STATION,',error[1], error[0], error[2]
        dist_set = set(pose_dist_list)
        if len(dist_set) > 0:
            first_candidate_ind = pose_dist_list.index(sorted(dist_set)[0])            
            if(reached_station != self.stations_name[pose_ind_list[first_candidate_ind]]):
                print ('Next station,', self.stations_name[pose_ind_list[first_candidate_ind]])             
                return [self.stations[pose_ind_list[first_candidate_ind]], self.stations_name[pose_ind_list[first_candidate_ind]], self.stations_timer[pose_ind_list[first_candidate_ind]]]
            else:
                first_candidate_ind = pose_dist_list.index(sorted(dist_set)[1])
                print ('Next station,', self.stations_name[pose_ind_list[first_candidate_ind]])             
                return [self.stations[pose_ind_list[first_candidate_ind]], self.stations_name[pose_ind_list[first_candidate_ind]], self.stations_timer[pose_ind_list[first_candidate_ind]]]
        else:
            print ('No station candidate')
            return []

    def find_next_waypoint(self,pose):
        pose_dist_list = []
        pose_heading_list = []
        pose_ind_list = []
        pose_error_list = []

        twist = Twist()

        print ('No of Waypoint : ', len(self.waypoints))

        for i in range(0,len(self.waypoints)):
            error = self.find_direction(self.waypoints[i])
            #if(abs(error[0]) < 90) and (abs(error[1]) <= 3) and (abs(error[1]) >= 0.1)and (abs(error[2]) < 90):
            if(abs(error[0]) < 90) and (abs(error[1]) <= 3) and (abs(error[1]) >= 0.2)and (abs(error[2]) < 90):
                pose_heading_list.append(error[0])
                pose_dist_list.append(error[1])                
                pose_error_list.append(error[2])
                pose_ind_list.append(i)
                rospy.logwarn("heading, dist, point: {0} {1} {2}".format(error[0], error[1], error[2]))
            
        dist_set = set(pose_dist_list)
        print ('WP SET', len(dist_set))           
        if len(dist_set) > 0:

            if len(dist_set) == 1:
                #print 'Two candidate'
                first_waypoint = pose_dist_list.index(sorted(dist_set)[0])
                second_waypoint = first_waypoint                
            else :
                #print 'One candidate'
                first_waypoint= pose_dist_list.index(sorted(dist_set)[0])
                second_waypoint = pose_dist_list.index(sorted(dist_set)[1])

            print ('first_waypoint {0}'.format(first_waypoint))           
            print ('second_waypoint {0}'.format(second_waypoint))

            heading_error = pose_heading_list[first_waypoint]
            dist_error = pose_dist_list[first_waypoint]
            print ('heading_error, dist_error : ', pose_heading_list[first_waypoint], pose_dist_list[first_waypoint])

            p_error = heading_error/90
            print ('p_error : ', p_error)

            if (heading_error) > 0.2:
                if dist_error > 0.15:
                    twist.linear.x = 0.3
                else:
                    twist.linear.x = 0.1
                twist.angular.z = 0.45 * p_error
                #print 'turn_left'
            elif (heading_error) < -0.2:
                if dist_error > 0.15:
                    twist.linear.x = 0.3
                else:
                    twist.linear.x = 0.1
                twist.angular.z = 0.45 * p_error
                #print 'turn_right'
            else:
                if dist_error > 0.15:
                    twist.linear.x = 0.3
                else:
                    twist.linear.x = 0.2
                twist.angular.z = 0
                #print 'go forward'
              
            self.set_pub.publish(twist)
       
        else:
            print ('No waypoint candidate')
            rospy.logwarn('No next waypoint')
            return []

    def get_waypoint(self,req):
        ret = GetWayPoint._response_class()
        #print "REQUEST STATION", data.is_station
        if req.is_station:
            data = self.find_next_station(self.current_pose_tf,req.reached_station)
        else:
            data = self.find_next_waypoint(self.current_pose_tf)
            
        if len(data)==0:
            ret.waypoint = Pose()
            ret.is_stop = True
            ret.is_feasable = False
        else:
            ret.waypoint = data[0]
            ret.is_stop = True
            ret.is_feasable = True
            ret.name = data[1]
            ret.timer = data[2]
            #print self.current_pose_tf
        return ret
    
    def get_waypoint_arr(self,req):
        ret = GetWayPoint._response_class()
        rospy.logwarn("REQUEST FROM STATION {0}".format(req.reached_station))
        data = self.find_next_station_arr(req.reached_station)
        if len(data)==0:
            ret.waypoint = Pose()
            ret.is_stop = True
            ret.is_feasable = False
            rospy.logwarn("NO CANDIDATE")       
        else:
            ret.waypoint = data[0]
            ret.is_stop = True
            ret.is_feasable = True
            ret.name = data[1]
            ret.timer = data[2]
            #print self.current_pose_tf
        return ret
       
    def pose_cb(self,data):
        self.current_pose_tf = data
        rot = []
        rot.append(self.current_pose_tf.orientation.x)
        rot.append(self.current_pose_tf.orientation.y)
        rot.append(self.current_pose_tf.orientation.z)
        rot.append(self.current_pose_tf.orientation.w)
     
        current_heading = euler_from_quaternion(rot)
        self.current_heading_yaw = current_heading[2] * 180/math.pi
        if self.current_heading_yaw < 0:
                self.current_heading_yaw = 360 + self.current_heading_yaw

    def __init__(self, initial_state=''):
        rospy.init_node('destination_scheduler', anonymous=False)
        rate = rospy.Rate(10)
        self.is_loop = rospy.get_param('~loop', default=False)
        self.initial_state = rospy.get_param('~initial_state', default=0)
        self.draw_markerswaypoints_index = self.initial_state - 1
        self.current_pose_tf = Pose()
        self.current_heading_yaw = 0
        #print 'waypoints load with initial index',self.waypoints_index
        if initial_state != '':
            self.initial_state = initial_state
        self.csv_path = rospy.get_param(
            '~csv_path', default='/home/minirw/amv_ws/src/clmr_connect/waypoints/path_waypoints_list.csv')
        self.csv_station_path = rospy.get_param(
            '~csv_path', default='/home/minirw/amv_ws/src/clmr_connect/waypoints/stations_list.csv')
        
        self.import_param_waypoint_csv(self.csv_path)
        self.import_param_station_csv(self.csv_station_path)
        
        self.init_waypoint_markers()
        self.draw_markers()
        self.pose_sub = rospy.Subscriber("current_pose_tf", Pose, self.pose_cb)
        self.set_pub = rospy.Publisher("nav_vel", Twist, queue_size=1)
        
        #self.serv_getwaypoint = rospy.Service('get_waypoint', GetWayPoint, self.get_waypoint)
        #self.serv_getwaypoint_arr = rospy.Service('get_waypoint_arr', GetWayPoint, self.get_waypoint_arr)
        
        while not rospy.is_shutdown(): 
            self.marker_pub.publish(self.waypoint_marker_array)

            self.find_next_waypoint(self.current_pose_tf)

            rate.sleep()
        
if __name__ == '__main__':            
    WayPointScheduler()
    

