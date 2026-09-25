#!/usr/bin/env python3
import rospy
import math
import csv
import time
from geometry_msgs.msg import Pose, Point
from visualization_msgs.msg import Marker, MarkerArray

class stationPlotting():
    stations = []
    stations_name = []
    stations_mode = []
    stations_timer = []
    stations_pin = []     
    stations_count = 0
    stations_index = 0
    current_station_index = -1

    def read_station_csv(self, path):
        print ('path', path)
        with open(path, 'r') as file:
            reader = csv.reader(file, delimiter = ',')
            print ('station plotting is readed')
            for row in reader:
                
                if row[0] != 'name':    #check header name --> name,mode,timer,pin,x,y,z,qx,qy,qz,qw
                    print (row[0])
                    self.stations_name.append(row[0])
                    self.stations_mode.append(row[1])
                    self.stations_timer.append(int(row[2]))
                    self.stations_pin.append(row[3])

                    pose = Pose()
                    pose.position.x = float(row[4])
                    pose.position.y = float(row[5])
                    pose.position.z = float(row[6])
                    pose.orientation.x = float(row[7])
                    pose.orientation.y = float(row[8])
                    pose.orientation.z = float(row[9])
                    pose.orientation.w = float(row[10])
                    self.stations.append(pose)

    def add_station_name(self, pose, name):
        station_marker_name = Marker()
        station_marker_name.ns = 'stations_name'
        station_marker_name.id = self.id
        self.id = self.id + 1
        station_marker_name.type = Marker.TEXT_VIEW_FACING
        station_marker_name.action = Marker.ADD
        station_marker_name.lifetime = rospy.Duration(0)
        station_marker_name.scale.z = 0.5   #Text size
        station_marker_name.color.r = 0
        station_marker_name.color.g = 0
        station_marker_name.color.b = 0
        station_marker_name.color.a = 1
        station_marker_name.header.frame_id = 'map'
        station_marker_name.header.stamp = rospy.Time.now()
        station_marker_name.pose = pose
        station_marker_name.text = name
        return station_marker_name
    
    def init_station_markers(self):        
        # Set up our station markers
        marker_lifetime = 0  # 0 is forever
        marker_ns_l = 'stations_line'
        marker_ns_c = 'stations_circle'
        marker_ns_n = 'stations_name'

        self.id = 0
        marker_id = 0
        marker_line_color = {'r': 0.0, 'g': 0.0, 'b': 1.0, 'a': 1.0}
        marker_circle_color = {'r': 1.0, 'g': 0.0, 'b': 0.0, 'a': 1.0}    

        self.station_marker_array = MarkerArray()
       
        # Initialize the marker points list.
        self.station_markers_line = Marker()
        self.station_markers_line.ns = marker_ns_l
        self.station_markers_line.id = marker_id
        self.station_markers_line.type = Marker.LINE_LIST
        self.station_markers_line.action = Marker.ADD
        self.station_markers_line.lifetime = rospy.Duration(marker_lifetime)
        self.station_markers_line.scale.x = 0.04 #Path size
        self.station_markers_line.scale.y = 0.04
        self.station_markers_line.color.r = marker_line_color['r']
        self.station_markers_line.color.g = marker_line_color['g']
        self.station_markers_line.color.b = marker_line_color['b']
        self.station_markers_line.color.a = marker_line_color['a']

        self.station_markers_line.header.frame_id = 'map'
        self.station_markers_line.header.stamp = rospy.Time.now()
        self.station_markers_line.points = list()

        self.station_markers_circle = Marker()
        self.station_markers_circle.ns = marker_ns_c
        self.station_markers_circle.id = marker_id
        self.station_markers_circle.type = Marker.SPHERE_LIST
        self.station_markers_circle.action = Marker.ADD
        self.station_markers_circle.lifetime = rospy.Duration(marker_lifetime)
        self.station_markers_circle.scale.x = 0.5   #station circle size
        self.station_markers_circle.scale.y = 0.5
        self.station_markers_circle.scale.z = 0.5
        self.station_markers_circle.color.r = marker_circle_color['r']
        self.station_markers_circle.color.g = marker_circle_color['g']
        self.station_markers_circle.color.b = marker_circle_color['b']
        self.station_markers_circle.color.a = marker_circle_color['a']
        self.station_markers_circle.header.frame_id = 'map'
        self.station_markers_circle.header.stamp = rospy.Time.now()
        self.station_markers_circle.points = list()

        self.station_marker_name = Marker()
        self.station_marker_name.ns = marker_ns_n
        self.station_marker_name.id = marker_id
        self.station_marker_name.type = Marker.TEXT_VIEW_FACING
        self.station_marker_name.action = Marker.ADD
        self.station_marker_name.lifetime = rospy.Duration(marker_lifetime)
        self.station_marker_name.scale.z = 1
        self.station_marker_name.color.r = marker_line_color['r']
        self.station_marker_name.color.g = marker_line_color['g']
        self.station_marker_name.color.b = marker_line_color['b']
        self.station_marker_name.color.a = marker_line_color['a']
        self.station_marker_name.header.frame_id = 'map'
        self.station_marker_name.header.stamp = rospy.Time.now()
        self.station_marker_name.points = list()
 
    def draw_markers(self):
        for i in range(0, len(self.stations_name)):
            station_marker_name_new = self.add_station_name(self.stations[i], self.stations_name[i])
            self.station_marker_array.markers.append(station_marker_name_new)
        
        for i in range(0, len(self.stations)-1):
            p1 = Point()
            p2 = Point()
            p1 = self.stations[i].position
            p2 = self.stations[i+1].position
            self.station_markers_line.points.append(p1)
            self.station_markers_line.points.append(p2)
            self.station_markers_circle.points.append(p1)
        p1 = Point()
        p2 = Point()
        p1 = self.stations[len(self.stations)-1].position
        p2 = self.stations[0].position
        self.station_markers_line.points.append(p1)
        self.station_markers_line.points.append(p2)
        self.station_markers_circle.points.append(p1)       
        self.station_marker_array.markers.append(self.station_markers_line)
        self.station_marker_array.markers.append(self.station_markers_circle)

    def __init__(self):
        rospy.init_node('stations_plotting', anonymous=False)

        self.csv_path = rospy.get_param('~csv_path', default='/home/minirw/amv_ws/src/amv_service/station/station_list.csv')

        self.marker_pub = rospy.Publisher('station_markers_p2p', MarkerArray, queue_size=5)
        self.read_station_csv(self.csv_path)        
        self.init_station_markers()
        self.draw_markers()        
  
        rate = rospy.Rate(5)
        
        while not rospy.is_shutdown(): 
            self.marker_pub.publish(self.station_marker_array)            
            rate.sleep()
        
if __name__ == '__main__':            
    stationPlotting()
    

