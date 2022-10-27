#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from motion_msgs.msg import ManualControl
from motion_msgs.msg import alg_msgs
from motion_msgs.msg import lat_msgs
from motion_msgs.msg import dyn_msgs
from motion_msgs.msg import alle_msgs
import numpy as np
import pandas as pd
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


import math

def publish(): 
	alle = alle_msgs()
	alle.Velocity = 0
	alle.Acc = 0
	alle.Steering = 0 
	rospy.loginfo(alle) 
	publisher.publish(alle)    


def Subscriber():
	
	rospy.Subscriber('dummy_pub_gear_vel', alg_msgs,callback_function)
	rospy.Subscriber('/pedal_pos', ManualControl,callback_function_1)
	rospy.spin()
	
def callback_function(message):
        global v,a_l
        v = message.Velocity
        a_l = message.Acc
        
def callback_function_1(message):
	
	global Steering_Input
	Steering_Input = message.Steering
	alle = alle_msgs()
	alle.Velocity = v
	alle.Acc = a_l
	alle.Steering = Steering_Input
	rospy.loginfo(alle) 
	publisher.publish(alle)
	
	
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node("T2")
    publisher = rospy.Publisher('alle_inputs', alle_msgs,queue_size=10)
    for i in range(800):
    	publish()
    Subscriber()
    

if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	
	

        

