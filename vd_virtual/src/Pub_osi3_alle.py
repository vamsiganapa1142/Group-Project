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
from geometry_msgs.msg import Vector3
from osi3_bridge.msg import TrafficUpdateMovingObject

import numpy as np


def Subscriber():
	rospy.Subscriber('lateral_dynamics',lat_msgs,callback_function)
	rospy.spin()	

def callback_function(message):
	
	global posx,posy,velx,vely,accx,accy,yaw
	posx = message.xc
	posy = message.yc
	velx = message.xc_dot
	vely = message.yc_dot
	accx = message.xc_dot_dot
	accy = message.yc_dot_dot
	yaw = message.theta
	Vec3 = TrafficUpdateMovingObject()
	Vec3.object.position.x = posx
	Vec3.object.position.y = posy
	Vec3.object.velocity.x = velx
	Vec3.object.velocity.y = vely
	Vec3.object.velocity.z = 0
	Vec3.object.acceleration.x = accx
	Vec3.object.acceleration.y = accy
	Vec3.object.acceleration.z = 0
	Vec3.object.orientation.roll = 0
	Vec3.object.orientation.yaw = yaw
	Vec3.object.orientation.pitch = 0
	pub.publish(Vec3)
	rospy.loginfo(Vec3) 
    
    
if __name__ == "__main__":
	rospy.init_node("pub_osi3_alle")
	pub = rospy.Publisher('ego_data',TrafficUpdateMovingObject,queue_size=10)
	Subscriber()