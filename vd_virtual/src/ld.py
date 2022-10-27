#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Float32MultiArray
from motion_msgs.msg import ManualControl
from motion_msgs.msg import alg_msgs
import numpy as np
import pandas as pd
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios

import numpy as np

import sys
import numpy as np
import math

class Parameters():

  def __init__(self):
    
    self.sample_time = 0.01

    #=====================================
    # Coefficients of Engine
    #=====================================  
    self.a_0 = 533
    self.a_1 = 0.1
    self.a_2 = -0.0002
    self.d_0 = 3

    #=====================================
    # Tire Data
    #=====================================
    self.c = 406884                                    #Tire Stifness
    self.F_max = 5440                                  #Max Load at Tire
  
    #=====================================
    # Rolling Resistance Parameters
    #=====================================
    self.fr = 0.015                                    #Road Friction Coefficient
    self.m= 1644                                       #Vehicle Mass
    self.g= 9.81                                       #Acceleration Due to Gravity
    self.alpha_longitudinal = 0                        #Longitudinal Road Inclination
    self.alpha_lateral = 0                             #Later Road Inclination

    #=====================================
    # Air Resistance Parameters
    #=====================================
    self.Cd = 0.5                                      #Air Drag Coefficient
    self.area = 2.057                                  #Front Area of Car
    self.row = 1.2                                     # Air Dencity 

    #=====================================
    # Intertial Resistance Parameters 
    #=====================================
    self.Iw = 0.953                                    # Wheel Inertia Kgm^2
    self.Ia = 0.135                                    # Axel Inertia Kgm^2
    self.Gd = 1                                        # Differential Gear Ratio
    self.Ig = 0.01                                     # Gear Box Inertia Kgm^2
    self.b_1 = 7                                       # Acceleration / Deaceleration Calib Coefficient 
    self.b_2 = 1.5                                     # Break Calib Coefficient
    self.Ie = 0.227                                    # Engine Inertia Kgm^2
    self.r_stat = 0.327                                # Tire Radius Static m
    self.r_dyn  = 0.321                                # Tire Radius Dynamic m
    self.n_dr = 0.95

def torque(t,W_e):
  P = Parameters()
  #-----Engine Torque -------------------------#
  T_e = t * (P.a_0 + P.a_1 * W_e + P.a_2 * W_e * W_e) #Engine Torque with respect to throttle posistion
  return T_e

def dragtorque(t_e):
    P = Parameters()
    T_e = t_e - P.d_0
    # print(T_e)
    return T_e

def Acceleration(T_e,Vi,Ai,W_e,x,Gr):
  P = Parameters()
  #++++++++++++++++++++++++++++++++++++++++++
  #    LONGITUDINAL DYNAMIC CALCULATION
  #++++++++++++++++++++++++++++++++++++++++++  
  
  #-----Drive Line Inertias--------------------#
  k = 1 + ( ( (P.Iw+P.Ia) + ((P.Gd**2) * P.Ig ) + ((P.Gd**2) * (Gr**2) * P.Ie) ) / P.m * P.r_stat * P.r_dyn)   # Powertrain Inertial Calculation
  J_e = P.m * P.r_dyn * P.r_stat * (k - 1)            #Total Powertrain Intertia

  #--------Resistive Force on Vehicle----------#
  Frr = P.fr * P.m * P.g * np.cos(P.alpha_longitudinal) # Rolling Resistance
  Fri = P.m * P.g * np.sin(P.alpha_lateral)           # Inclination Resistance
  Fra = 0.5 * P.row * P.Cd * P.area * (Vi**2)         # Air Resistance
  Frac = k * P.m * Ai                             # Intertial Resistance

  F_load = Frr + Fri + Fra + Frac                # Total Resistance Acting 

  #-----------Angular Speed of Engine----------#
  W_e_dot = (T_e * 2.7 - (Gr * P.r_dyn * F_load)) / J_e #Engine Angular Acceleration

  W_e = W_e + W_e_dot * P.sample_time             #Engine Angular Speed
  #print(W_e)

  #----------Driving Force of Vehicle----------#
  W_w = W_e / Gr                                #Wheel Angular Speed
  s  = (W_w * P.r_dyn - Vi) / Vi                  #Wheel Slip
  #print(s)
  cs = P.c * s                                   #Total Force on Wheels
  
  F_tr = (P.n_dr * T_e * P.Gd * Gr) / P.r_stat

  #-----------Total Force on Vehicle-----------#
  F_Velocity = (F_tr * P.b_1 - F_load)                             # Total Force 
 
  #----------Vehicle Acceleration--------------#
  Acc = F_Velocity / P.m 

  #----------Vehicle Velocity------------------#
  vel = Velocity(Vi,Acc,0.01)
  
  return vel,Acc,W_e

def Deceleration(T_e,Vi,Ai,W_e,x,Gr):  
  P = Parameters()
  #++++++++++++++++++++++++++++++++++++++++++
  #    LONGITUDINAL DYNAMIC CALCULATION
  #++++++++++++++++++++++++++++++++++++++++++ 
  
  #-----Drive Line Inertias--------------------#
  k = 1 + ( ( (P.Iw+P.Ia) + ((P.Gd**2) * P.Ig ) + ((P.Gd**2) * (Gr**2) * P.Ie) ) / P.m * P.r_stat * P.r_dyn)   # Powertrain Inertial Calculation
  J_e = P.m * P.r_dyn * P.r_stat * (k - 1)            #Total Powertrain Intertia

  #--------Resistive Force on Vehicle----------#
  Frr = P.fr * P.m * P.g * np.cos(P.alpha_longitudinal) # Rolling Resistance
  Fri = P.m * P.g * np.sin(P.alpha_lateral)           # Inclination Resistance
  Fra = 0.5 * P.row * P.Cd * P.area * (Vi**2)         # Air Resistance
  Frac = k * P.m * Ai                             # Intertial Resistance

  F_load = Frr + Fri + Fra - Frac               # Total Resistance Acting 

  F_tr = (P.n_dr * T_e * P.Gd * Gr) / P.r_stat       # Driving Force Required

  #-----------Angular Speed of Engine----------#
  W_e_dot = (T_e - (Gr * P.r_dyn * F_load)) / J_e #Engine Angular Acceleration

  W_e = W_e + W_e_dot * P.sample_time             #Engine Angular Speed

  #-----------Total Force on Vehicle-----------#
  F_Velocity = F_tr - F_load                             # Total Force 
 
  #----------Vehicle Acceleration--------------#
  Acc = P.b_1 * F_Velocity / P.m 

  #----------Vehicle Velocity------------------#
  vel = Velocity(Vi,Acc,0.01)
  
  return vel,Acc,W_e

def Brakes(B_p,Vi,Ai,W_e,x,Gr):  
  P = Parameters()
  #++++++++++++++++++++++++++++++++++++++++++
  #    LONGITUDINAL DYNAMIC CALCULATION
  #++++++++++++++++++++++++++++++++++++++++++

  #-----Break Torque -------------------------#
  P_f = 500 * B_p
  Sys_p = 0.4 * P_f

  BT_F = 10.25 * Sys_p
  BT_R = 5.125 * Sys_p
  
  #-----Drive Line Inertias--------------------#
  k = 1 + ( ( (P.Iw+P.Ia) + ((P.Gd**2) * P.Ig ) + ((P.Gd**2) * (Gr**2) * P.Ie) ) / P.m * P.r_stat * P.r_dyn)   # Powertrain Inertial Calculation

  #--------Resistive Force on Vehicle----------#
  Frr = P.fr * P.m * P.g * np.cos(P.alpha_longitudinal) # Rolling Resistance
  Fri = P.m * P.g * np.sin(P.alpha_lateral)           # Inclination Resistance
  Fra = 0.5 * P.row * P.Cd * P.area * (Vi**2)         # Air Resistance
  Frac = k * P.m * Ai                             # Intertial Resistance

  F_load = Frr + Fri + Fra + Frac               # Total Resistance Acting 
  
  #-----------Total Brake Force on Vehicle-----------#
  F_f = BT_F / P.r_dyn                            # Total Force 
  F_r = BT_R / P.r_dyn

  T_Force = (F_f * 2) + (F_r * 2)
 
  #----------Vehicle Acceleration--------------#
  Acc = -(P.b_2 * T_Force + F_load)  / P.m 

  #----------Vehicle Velocity------------------#
  vel = Velocity(Vi,Acc,0.01)
  
  return vel,Acc

def Velocity(Vi,Acc,sample_time):
  Vel = Vi+ Acc * sample_time                           #Vehicle Velocity
  return Vel

def Posistion(Vel,x,sample_time):
  x = x + Vel * sample_time                              #Vehicle Posistion on x-axis 
  return x

#===============================================
#               Initial State
#===============================================
def publish():
  Array = alg_msgs()
  Array.Velocity = 0
  Array.Gear = 3.13
  publisher.publish(Array)
  rospy.loginfo(Array)
  
def Nextstate(gp,bp,vel,gear):
  Acc = 0
  min_Vel = 0
  Vel = vel
  Pos = 0
  W_e = 92.45
  max_drag_torque = -160.39
  T_e = max_drag_torque
  Gear = gear
  G_p = gp
  B_p = bp

  if (G_p > 0):
    T_e = torque(G_p, W_e)
    Vel,Acc,W_e = Acceleration(T_e,Vel,Acc,W_e,Pos,Gear)
    Pos = Posistion(Vel,Pos,0.01)
  else:
    T_e = dragtorque(T_e)
    Vel,Acc,W_e = Deceleration(T_e,Vel,Acc,W_e,Pos,Gear)
    Pos = Posistion(Vel,Pos,0.01)
    if (T_e <= -160.39):
      T_e = max_drag_torque
    if (Vel < 0.1):
      Vel = min_Vel
    if (B_p > 0):
      Vel,Acc = Brakes(B_p,Vel,Acc,W_e,Pos,Gear)
    if (Vel < 0.1):
      Vel = min_Vel
  
  if (Vel < 14):
    Gear = 3.13

  if (Vel > 14):
    Gear = 2.59

  if (Vel > 18.3):
    Gear = 1.96

  if (Vel > 25.4):
    Gear = 1.24

  if (Vel > 38):
    Gear = 0.98

  if (Vel > 45.2):
    Gear = 0.91

  if (Vel > 47.6):
    Gear = 0.84

  T_e = checktorqueLimit(T_e) 
  Vel = checkVelocityLimit(Vel)
  Acc = Acc
  Gear = Gear 
  return Vel,Gear,Acc
  
def torque_limit(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def checktorqueLimit(T_e):
    torque = torque_limit(T_e, -161.39, 545)
    return torque

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input
    return input

def checkVelocityLimit(pos):
    pos = constrain(pos, 0, 85)
    return pos

        
def Subscriber():
  rospy.Subscriber('gear', Float32,callback_function)
  rospy.Subscriber('vel',Float32,callback_function_1)
  rospy.Subscriber('/pedal_pos', ManualControl,callback_function_2)
  rospy.spin()

def callback_function(message):
  global Gear_1
  gear = message.data
  Gear_1 = gear
  
def callback_function_1(message):
  global Velocit
  vel = message.data
  Velocit = vel

def callback_function_2(message):
  
  global pub1
  pub1 = message.Brake
  pub2 = message.Throttle
  
  rate = rospy.Rate(100)
  Vel,Gear,Acc = Nextstate(pub2,pub1,Velocit,Gear_1)
  Acc = float(Acc)
  Vel = float(Vel)
  Gear = float(Gear)
  Alg_msg = alg_msgs()
  Alg_msg.Gear = Gear
  Alg_msg.Velocity = Vel
  Alg_msg.Acc=Acc
  rospy.loginfo(Gear)
  rospy.loginfo(Vel)
  publisher.publish(Alg_msg)
  
  
if __name__=="__main__":
    if os.name != 'nt':
    
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node("Team_2")
    publisher = rospy.Publisher('dummy_pub_gear_vel', alg_msgs,queue_size=10)
    for i in range(200):
      publish()
    Subscriber()
    
  
    

if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
