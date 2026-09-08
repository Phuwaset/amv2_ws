#!/usr/bin/env python3
import rospy
import math
import csv
import time
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray
from amv_navigation.msg import WaypointsData
from amv_navigation.srv import ReadWaypoints

class WayPointPlotting():    
    #waypoints_data = WaypointsData()
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
                    #self.waypoints_data.name = row[0]
                    #self.waypoints_data.mode = row[1]
                    #self.waypoints_data.timer = int(row[2])
                    #self.waypoints_data.pin = row[3]

                    pose = WaypointsData()
                    pose.name = row[0]
                    pose.mode = row[1]
                    pose.timer = int(row[2])
                    pose.pin = row[3]
                    pose.x = float(row[4])
                    pose.y = float(row[5])
                    pose.z = float(row[6])
                    pose.qx = float(row[7])
                    pose.qy = float(row[8])
                    pose.qz = float(row[9])
                    pose.qw = float(row[10])
                    self.waypoints.append(pose)
        
        #print (self.waypoints) 
 
    def handle_read_waypoints(self, req):
        ret = ReadWaypoints._response_class()             
        if len(self.waypoints) > 0:
            status = 200    #Normal code
        else:
            status = 400    #Fault code
        ret.status_code = status
        ret.waypoints_data = self.waypoints

        return ret


    def __init__(self):
        rospy.init_node('waypoints_plotting', anonymous=False)

        self.csv_path = rospy.get_param('~csv_path', default='/home/minirw/amv_ws/src/amv_navigation/waypoints/waypoints_list.csv')

        self.read_waypoint_csv(self.csv_path)   

        s = rospy.Service('read_waypoints', ReadWaypoints, self.handle_read_waypoints)     

        #self.draw_markers()        
  
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown(): 
            #self.marker_pub.publish(self.waypoint_marker_array)            
            rate.sleep()
        
if __name__ == '__main__':            
    WayPointPlotting()
    

