#!/usr/bin/env python3

import rospy
import actionlib
from actionlib import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist, PoseWithCovarianceStamped
#from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker, MarkerArray
from math import  pi
from collections import OrderedDict
from smach import State, StateMachine
from geometry_msgs.msg import Twist
import threading
import sys
from tf import TransformListener
from move_base_msgs.msg import MoveBaseActionGoal, MoveBaseGoal

class WayPointTeaching:

    def add_name(self,pose,name):
        waypoint_marker_name = Marker()
        waypoint_marker_name.ns = 'waypoints_name'
        waypoint_marker_name.id = self.id
        self.id = self.id +1
        waypoint_marker_name.type = Marker.TEXT_VIEW_FACING
        waypoint_marker_name.action = Marker.ADD
        waypoint_marker_name.lifetime = rospy.Duration(0)
        waypoint_marker_name.scale.z = 0.5
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
        marker_lifetime = 0  # 0 is forever
        marker_ns_l = 'waypoints_line'
        marker_ns_c = 'waypoints_circle'
        marker_ns_n = 'waypoints_name'

        self.id = 0
        marker_id = 0
        marker_line_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}
        marker_circle_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}    

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
        self.waypoint_markers_circle.scale.x = 0.3   #Waypoint circle size
        self.waypoint_markers_circle.scale.y = 0.3
        self.waypoint_markers_circle.scale.z = 0.3
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

    def add_waypoint_callback(self,data):
        #print (data)
        self.waypoints.append(data.goal.target_pose.pose)
        #print (self.waypoints)
        self.waypoints_name.append(self.waypoints_name_prefix+str(self.waypoints_count))
        self.waypoints_count = self.waypoints_count + 1
        self.init_waypoint_markers()
        if len(self.waypoints) > 1:
            for i in range(0,len(self.waypoints)-1):           
                # p1 = Point()
                # p2 = Point()
                p1 = self.waypoints[i].position
                p2 = self.waypoints[i+1].position
                self.waypoint_markers_line.points.append(p1)
                self.waypoint_markers_line.points.append(p2)
                self.waypoint_markers_circle.points.append(p1)
                waypoint_marker_name_new = self.add_name(self.waypoints[i],self.waypoints_name[i])
                self.waypoint_marker_array.markers.append(waypoint_marker_name_new)
            self.waypoint_marker_array.markers.append(self.waypoint_markers_line)
            self.waypoint_marker_array.markers.append(self.waypoint_markers_circle)

            p1 = self.waypoints[len(self.waypoints)-1].position
            p2 = self.waypoints[0].position
            self.waypoint_markers_line.points.append(p1)
            self.waypoint_markers_line.points.append(p2)
            self.waypoint_markers_circle.points.append(p1)
            waypoint_marker_name_new = self.add_name(self.waypoints[len(self.waypoints)-1],self.waypoints_name[len(self.waypoints)-1])
            self.waypoint_marker_array.markers.append(waypoint_marker_name_new)

        else:
            self.waypoint_markers_circle.points.append(self.waypoints[0].position)
            self.waypoint_marker_array.markers.append(self.waypoint_markers_circle)
            waypoint_marker_name_new = self.add_name(self.waypoints[0],self.waypoints_name[0])
            self.waypoint_marker_array.markers.append(waypoint_marker_name_new)
        
        param_str = 'name,mode,timer,pin,x,y,z,qx,qy,qz,qw\n'
        for i in range(0,len(self.waypoints)):
            param_str = param_str + str(self.waypoints_name[i]) + ','
            param_str = param_str + 'A' + ','
            param_str = param_str + '1' + ','
            param_str = param_str + 'D' + ','
            param_str = param_str + str(self.waypoints[i].position.x) + ','
            param_str = param_str + str(self.waypoints[i].position.y) + ','
            param_str = param_str + str(self.waypoints[i].position.z) + ','
            param_str = param_str + str(self.waypoints[i].orientation.x) + ','
            param_str = param_str + str(self.waypoints[i].orientation.y) + ','
            param_str = param_str + str(self.waypoints[i].orientation.z) + ','
            param_str = param_str + str(self.waypoints[i].orientation.w) + '\n'
          
        with open(self.csv_path, 'w') as outfile:
            outfile.write(param_str)
            rospy.loginfo('%s is added to waypoints list',(self.waypoints_name[i])) 

    def __init__(self):
        rospy.init_node('waypoint_teaching', anonymous=False)
        
        self.csv_path = rospy.get_param('~csv_path',default='/home/minirw/amv_ws/src/amv_navigation/waypoints/waypoints_list.csv')
         # Define a marker publisher.
        self.marker_pub = rospy.Publisher('waypoint_markers_p2p', MarkerArray, queue_size=5)
        self.waypoints = []
        self.waypoints_name = []
        self.waypoints_name_prefix = 'WP'
        self.waypoints_count = 1
        self.waypoint_marker_array = MarkerArray()
        way_sub = rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.add_waypoint_callback) 

        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): 
            self.marker_pub.publish(self.waypoint_marker_array)
            rate.sleep()
        rospy.loginfo('Waypoint teaching terminated')       

    def shutdown(self):
        rospy.loginfo("Exit..")
        
if __name__ == '__main__':
    WayPointTeaching()

