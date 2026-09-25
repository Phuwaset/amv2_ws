#!/usr/bin/env python3
import rospy
import tf
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import Twist, Pose
#from nav_msgs.msg import Odometry
from math import pi, sqrt, atan2

#WAYPOINTS = [[0.5,0],[1,0],[1,0],[1,0.5],[1,1],[1,1],[0.5,1],[0,1],[0,1],[0,0.5],[0,0]] 
WAYPOINTS = [[-3.50,-0.78],[-1.29,-0.78]] 

class PID:
    """
    Discrete PID control
    """
    def __init__(self, P=0.0, I=0.0, D=0.0, Derivator=0, Integrator=0, Integrator_max=10, Integrator_min=-10):
        self.Kp = P
        self.Ki = I
        self.Kd = D
        self.Derivator = Derivator
        self.Integrator = Integrator
        self.Integrator_max = Integrator_max
        self.Integrator_min = Integrator_min
        self.set_point = 0.0
        self.error = 0.0

    def update(self, current_value):
        self.error = self.set_point - current_value
        if self.error > pi:  # specific design for circular situation
            self.error = self.error - 2*pi
        elif self.error < -pi:
            self.error = self.error + 2*pi
        self.P_value = self.Kp * self.error
        self.D_value = self.Kd * ( self.error - self.Derivator)
        self.Derivator = self.error
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
            self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
            self.Integrator = self.Integrator_min
        self.I_value = self.Integrator * self.Ki
        PID = self.P_value + self.I_value + self.D_value
        return PID

    def setPoint(self, set_point):
        self.set_point = set_point
        self.Derivator = 0
        self.Integrator = 0

    def setPID(self, set_P=0.0, set_I=0.0, set_D=0.0):
        self.Kp = set_P
        self.Ki = set_I
        self.Kd = set_D

class amv_move():
    def __init__(self):
        rospy.init_node('amv_move', anonymous=False)
        rospy.loginfo("Press CTRL + C to terminate")
        rospy.on_shutdown(self.stop)
        
        self.max_lin_vel = 0.4
        self.max_ang_vel = 0.4
        self.threshole_lin_vel = 0.1
        self.threshole_ang_vel = 0.07
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.pid_theta = PID(0,0,0)  # initialization

        #self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.amv_pose_sub = rospy.Subscriber("amv_pose_tf", Pose, self.amv_pose_callback)
        self.vel_pub = rospy.Publisher('nav_vel', Twist, queue_size=10)
        self.vel = Twist()
        self.rate = rospy.Rate(20)
        self.counter = 0
        self.trajectory = list()

        # track a sequence of waypoints
        for point in WAYPOINTS:
            print (point[0], point[1])
            self.move_to_point(point[0], point[1])            
            rospy.sleep(1)
        self.stop()
        rospy.logwarn("Action done.")

        # plot trajectory
        # data = np.array(self.trajectory)
        # np.savetxt('trajectory.csv', data, fmt='%f', delimiter=',')
        # plt.plot(data[:,0],data[:,1])
        # plt.show()


    def move_to_point(self, x, y):
        # Compute orientation for angular vel and direction vector for linear vel
        rospy.sleep(1)
        diff_x = x - self.x
        diff_y = y - self.y
        direction_vector = np.array([diff_x, diff_y])
        direction_vector = direction_vector/sqrt(diff_x*diff_x + diff_y*diff_y)  # normalization
        theta = atan2(diff_y, diff_x)
        print('dx = ', diff_y, 'dy = ', diff_x)
        # We should adopt different parameters for different kinds of movement
        self.pid_theta.setPID(0.8, 0, 0)     # P control while steering
        self.pid_theta.setPoint(theta)
        rospy.logwarn("### PID: set target theta = " + str(theta) + " ###")

        # Adjust orientation first
        while not rospy.is_shutdown():
            angular = self.pid_theta.update(self.theta)            
            if abs(angular) > self.max_ang_vel:
                angular = angular/abs(angular)*self.max_ang_vel
            if abs(angular) < self.threshole_ang_vel:
                break

            print(0, angular)
            self.vel.linear.x = 0
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        # Have a rest
        self.stop()
        self.pid_theta.setPoint(theta)
        #self.pid_theta.setPID(1, 0, 0)   # PI control while moving
        #self.pid_theta.setPID(1, 0.02, 0.2)  # PID control while moving
        self.pid_theta.setPID(0.5, 0.02, 0.2)  # PID control while moving

        # Move to the target point
        while not rospy.is_shutdown():
            diff_x = x - self.x
            diff_y = y - self.y
            vector = np.array([diff_x, diff_y])
            linear = np.dot(vector, direction_vector) # projection
            if abs(linear) > self.max_lin_vel:
                linear = linear/abs(linear)*self.max_lin_vel

            angular = self.pid_theta.update(self.theta)
            if abs(angular) > self.max_ang_vel:
                angular = angular/abs(angular)*self.max_ang_vel

            if abs(linear) < self.threshole_lin_vel and abs(angular) < self.threshole_ang_vel:
                break
            
            print(linear, angular)
            self.vel.linear.x = linear
            self.vel.angular.z = angular
            self.vel_pub.publish(self.vel)
            self.rate.sleep()

        self.stop()


    def stop(self):
        self.vel.linear.x = 0
        self.vel.angular.z = 0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)


    def amv_pose_callback(self,data):
        quarternion = [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.theta = yaw
        self.x = data.position.x
        self.y = data.position.y
        #print (self.x , self.y)

        # Make messages saved and prompted in 5Hz rather than 100Hz
        self.counter += 1
        if self.counter == 20:
            self.counter = 0
            self.trajectory.append([self.x,self.y])
            #rospy.loginfo("amv_pose: x=" + str(self.x) + ";  y=" + str(self.y) + ";  theta=" + str(self.theta))


if __name__ == '__main__':
    try:
        amv_move()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMV navigation action terminated.")
    

