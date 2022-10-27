#!/usr/bin/env python

import rospy
from motion_msgs.msg import ManualControl
from osi3_bridge.msg import TrafficUpdateMovingObject
from geometry_msgs.msg import Point
import math
import numpy as np
from numpy import linalg as la
import csv
import os

class pure_pursuit:

    def __init__(self):

        self.LOOKAHEAD_DISTANCE = 5.6 # meters
        self.THROTTLE = 0.5
        self.goal = 0
        self.read_waypoints()
        self.msg = ManualControl()
        self.msg.Throttle = 0.25
        self.wheel_base = 2.75

        # Publisher for 'ManualControl' (Throttle and steering angle)
        self.pub = rospy.Publisher('/pedal_pos', ManualControl, queue_size=1)

	#Publisher for the goal point
	self.goal_pub = rospy.Publisher('/waypoint/goal', Point, queue_size=1)

        rospy.Subscriber('ego_data', TrafficUpdateMovingObject, self.callback, queue_size=1)

    # Import waypoints.csv into a list (path_points)
    def read_waypoints(self):

        dirname  = os.path.dirname(__file__)
        filename = os.path.join(dirname, '/home/datta/catkin_ws/src/pure_pursuit_controller/waypoints/waypoints.csv')

        with open(filename) as f:
            path_points = [tuple(line) for line in csv.reader(f)]

        # Turn path_points into a list of floats to eliminate the need for casts in the code below.
        self.path_points_x   = [float(point[0]) for point in path_points]
        self.path_points_y   = [float(point[1]) for point in path_points]
        self.path_points_w   = [float(point[2]) for point in path_points]

        self.dist_arr= np.zeros(len(self.path_points_y))

   
    # Computes the Euclidean distance between two 2D points p1 and p2.
    def dist(self, p1, p2):
        return np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

    # Input data is PoseStamped message from topic /odometery.
    # Runs pure pursuit and publishes throttle and steering angle.
    def callback(self,data):

        rospy.loginfo(rospy.get_caller_id() + "Odometery reciving %s", data.data)
        yaw = data.object.orientation.yaw
        x = data.object.position.x
        y = data.object.position.y

        self.path_points_x = np.array(self.path_points_x)
        self.path_points_y = np.array(self.path_points_y)

        ## finding the distance of each way point from the current position 

        for i in range(len(self.path_points_x)):
            self.dist_arr[i] = self.dist((self.path_points_x[i],self.path_points_y[i]),(x,y))

        ##finding those points which are less than the look ahead distance (will be behind and ahead of the vehicle)

        goal_arr = np.where((self.dist_arr < self.LOOKAHEAD_DISTANCE)&(self.dist_arr > self.LOOKAHEAD_DISTANCE))[0]

        ##finding the goal point which is the last in the set of points less than the lookahead distance
        ##if the closest points array could not be formed, then the point which is closest to the current position is the goal. 
        
        for idx in goal_arr:
            v1 = [self.path_points_x[idx]-x , self.path_points_y[idx]-y]
            v2 = [np.cos(yaw), np.sin(yaw)]
            temp_angle = self.find_angle(v1,v2)
            if abs(temp_angle) < np.pi/2:
                self.goal = idx
                # print(self.goal)
                break

        ##finding the distance of the goal point from the vehicle coordinates

        L = self.dist_arr[self.goal]

        ##Transforming the goal point into the vehicle coordinate frame 

        gvcx = self.path_points_x[self.goal] - x
        gvcy = self.path_points_y[self.goal] - y 
        goal_x_veh_coord = gvcx*np.cos(yaw) + gvcy*np.sin(yaw)
        goal_y_veh_coord = gvcy*np.cos(yaw) - gvcx*np.sin(yaw)

        # math: find the curvature and the Steering angle 
        alpha = self.path_points_w[self.goal] - (yaw)
        k = 2 * math.sin(alpha)/L
        Steering_i = math.atan(k * self.wheel_base)

        Steering = Steering_i*2
        Steering = np.clip(Steering, -10.9955,10.9955) # 630 degrees  = 10.9955 radians because car take only max of 630 degrees.
        print(Steering)

        self.set_Throttle(Steering)
        self.const_Throttle(Steering)

	#publish the goal in the vehicle coordinates. 
	goalPoint = Point(float(goal_x_veh_coord),float(goal_y_veh_coord),float(Steering));
	self.goal_pub.publish(goalPoint)


    def send_command(self):
    	self.pub.publish(self.msg)

    # USE THIS FUNCTION IF CHANGEABLE Throttle IS NEEDED(Values need to be tuned and changed according to Team1&2) 
    def set_Throttle(self,Steering):
        if (abs(Steering)>0.5):
            self.LOOKAHEAD_DISTANCE = 5.6
            self.msg.Steering = Steering

            if self.msg.Throttle - 0.25 >= 0.15:
                self.msg.Throttle -= 0.1

        else:
            self.LOOKAHEAD_DISTANCE = 7.6
            self.msg.Steering = Steering

            if self.THROTTLE - self.msg.Throttle > 0.1:
                self.msg.Throttle += 0.1
            print(Steering,self.msg.Throttle)
        object.header.stamp = rospy.Time.now()
      

    # USE THIS FUNCTION IF CONSTANT SPEED IS NEEDED
    def const_speed(self,Steering):
        self.LOOKAHEAD_DISTANCE = 5.6
        self.msg.Steering = Steering
        self.msg.Throttle = self.THROTTLE
        

    # find the angle bewtween two vectors    
    def find_angle(self, v1, v2):
        cosang = np.dot(v1, v2)
        sinang = la.norm(np.cross(v1, v2))
        return np.arctan2(sinang, cosang)


if __name__ == '__main__':
    rospy.init_node('pure_pursuit')
    rospy.loginfo("pure_pursuit Controller is running %s" % rospy.get_time())
    
    C = pure_pursuit()  
    r = rospy.Rate(100)

    while not rospy.is_shutdown():
        C.send_command()
        r.sleep()
