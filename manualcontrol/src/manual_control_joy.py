#! /usr/bin/env python3

import rospy
import pygame
from motion_msgs.msg import ManualControl
import sys, select, os
import time

MAX_GEAR = 7
MIN_GEAR = 0

MAX_HANDBRAKE = 1
MIN_HANDBRAKE = 0

GEAR_STEP_SIZE = 1
HANDBRAKE_STEP_SIZE = 1


def postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake):
    return "currently:\tthrottle %s\t steering %s\t gear %s\t brake %s\t handbrake%s " % (target_throttle,target_steering,target_gear,target_brake,target_handbrake)

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input


def checkGearLimit(pos):
    pos = constrain(pos, MIN_GEAR, MAX_GEAR)
    return pos


def checkHandbrakeLimit(pos):
    pos = constrain(pos, MIN_HANDBRAKE, MAX_HANDBRAKE)
    return pos


pygame.init()

pygame.joystick.init()
clock = pygame.time.Clock()

rospy.init_node('manualcontroljoy')
pub = rospy.Publisher('/pedal_pos', ManualControl, queue_size=10)
autorepeat_rate = 100

target_throttle = 0.0
target_steering = 0.0
target_gear = 0.0
target_brake = 0.0
target_handbrake = 0.0
done = False 

motion = ManualControl()
motion.Throttle = target_throttle
motion.Steering = target_steering
motion.Gear = target_gear
motion.Brake = target_brake
motion.Handbrake = target_handbrake


while not done:
    pub.publish(motion)
    
    for event in pygame.event.get():

	    joystick_count = pygame.joystick.get_count()
	    
	    for i in range(joystick_count):
	    	joystick = pygame.joystick.Joystick(i)
	    	joystick.init()
	    	

	    	target_throttle = (joystick.get_axis(5)+1)/2
	    	print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	target_brake = (joystick.get_axis(2)+1)/2
	    	print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	target_steering = (joystick.get_axis(3))
	    	print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	if joystick.get_button(4) == True:
	    		target_gear = checkGearLimit(target_gear + GEAR_STEP_SIZE)
	    		print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	elif joystick.get_button(5) == True:
	    		target_gear = checkGearLimit(target_gear - GEAR_STEP_SIZE)
	    		print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	elif joystick.get_button(3) == True:
	    		target_handbrake = checkHandbrakeLimit(target_handbrake + HANDBRAKE_STEP_SIZE)
	    		print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))

	    	else:
	    		target_handbrake = 0
	    		print(postn(target_throttle, target_steering, target_gear, target_brake, target_handbrake))


	    	motion = ManualControl()
	    	motion.Throttle = target_throttle
	    	motion.Steering= target_steering
	    	motion.Gear = target_gear
	    	motion.Brake = target_brake
	    	motion.Handbrake= target_handbrake
	    	pub.publish(motion)

clock.tick(60)

pygame.quit

