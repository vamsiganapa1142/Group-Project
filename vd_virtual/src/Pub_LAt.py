#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from motion_msgs.msg import ManualControl
from motion_msgs.msg import alg_msgs
from motion_msgs.msg import lat_msgs
from motion_msgs.msg import dyn_msgs
import numpy as np
import pandas as pd
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


import math


def publish():
    dyn_msg = dyn_msgs()
    dyn_msg.xc = 0
    dyn_msg.yc = 0
    dyn_msg.theta = 0
    dyn_msg.theta_dot =0
    dyn_msg.yc_dot = 0
    dyn_msg.beta = 0
    dyn_msg.betadot = 0
    dyn_msg.xc_dot_dot = 0
    dyn_msg.yc_dot_dot = 0 
    dyn_msg.thetacc = 0
    dyn_msg.steer_prev = 0
    rospy.loginfo(dyn_msg)
    publisher.publish(dyn_msg)


def Subscriber():
	
	rospy.Subscriber('lateral_dynamics', lat_msgs,callback_function)
	
	rospy.spin()

def callback_function(message):
	
	global xc,yc,theta,beta,yc_dot,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev,Steering_Input
	xc= message.xc
	yc= message.yc
	yc_dot = message.yc_dot
	theta = message.theta
	beta = message.beta
	theta_dot = message.theta_dot
	betadot = message.betadot
	xc_dot_dot = message.xc_dot_dot
	yc_dot_dot = message.yc_dot_dot 
	thetacc = message.thetacc
	steer_prev = message.steer_prev
	dyn_msg = dyn_msgs()
	dyn_msg.xc = xc
	dyn_msg.yc = yc
	dyn_msg.theta = theta
	dyn_msg.theta_dot =theta_dot
	dyn_msg.yc_dot = yc_dot
	dyn_msg.beta = beta
	dyn_msg.betadot = betadot
	dyn_msg.xc_dot_dot = xc_dot_dot
	dyn_msg.yc_dot_dot = yc_dot_dot 
	dyn_msg.thetacc = thetacc
	dyn_msg.steer_prev = steer_prev
	publisher.publish(dyn_msg)
	
	
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node("Team_two")
    publisher = rospy.Publisher('dyn_dynamics', dyn_msgs,queue_size=10)
    for i in range(700):
    	publish()
    Subscriber()
    
	
    

if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

