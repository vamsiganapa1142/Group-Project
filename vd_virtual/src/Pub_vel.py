#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32
from std_msgs.msg import Int32MultiArray
from motion_msgs.msg import alg_msgs
import numpy as np

def Subscriber():
	rospy.Subscriber('dummy_pub_gear_vel', alg_msgs,callback_function)
	rospy.spin()	
	
	
def publish():
	Array = Float32()
	Array.data = 0
	pub.publish(Array)
	rospy.loginfo(Array) 

def callback_function(message):
	
	global vel 
	vel = message.Velocity
	message_to_publish = Float32()
	message_to_publish.data = vel
	pub.publish(message_to_publish)
	rospy.loginfo(vel)    
    
if __name__ == "__main__":
	rospy.init_node("pub_vel")
	pub = rospy.Publisher('vel', Float32,queue_size=10)
	for i in range(500):
    		publish()
	Subscriber()
	
