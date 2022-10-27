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

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import math
import numpy as np


class Parameters():
    def __init__(self):
      self.L = 2.750

      self.lr = 1.510

      self.delta_max = 10.99

      self.deltadot_max = 8.72
      
      self.sample_time = 0.01

      self.g= 10

      self.C_Sf = 21.92 / 1.0489

      self.C_Sr = 21.92 / 1.0489

      self.h=0.475

      self.m = 1644

      self.I = 1786.89716

      self.mu = 1.0489

      self.lf=1.240

      self.delta_min = -10.99

      self.MIN_THETA = -0.48

      self.MAX_THETA = 0.48

      self.r = 17
   

def delta_Fast(Steering_Input,steer_wheel):                                                   #steer_prev,xc,yc,theta,v,delta,a_l,deltadott):          
  P = Parameters()
  delta = steer_wheel / P.r 
  rospy.loginfo(delta)
  return delta

def deltadot_Fast(delta,steer_prev):
  P = Parameters()
  deltadot= (delta - steer_prev) / P.sample_time
  return deltadot



def beta_Fast(delta):
  P = Parameters()
  # ==================================
  #  Implement kinematic model here
  # ==================================
  #so that max rate is not exceeded    
  beta = np.arctan(P.lr * np.tan(delta) / P.L)
  return beta

def theta_dot_Fast(v,beta,delta):
  P = Parameters()
  theta_dot = (v * np.cos(beta) * np.tan(delta)) / P.L
  return theta_dot

def betadot_Fast(v,a_l,delta,beta,theta_dot):
  P = Parameters()
  betadot = (P.mu/v*(P.lr+P.lf))*(P.C_Sf*(P.g*P.lr-a_l*P.h)*delta-(P.C_Sr*(P.g*P.lf+a_l*P.h)+P.C_Sf*(P.g*P.lr-a_l*P.h))*beta + (P.C_Sr*(P.g*P.lf+a_l*P.h)*P.lr-P.C_Sf*(P.g*P.lr-a_l*P.h)*P.lf)*theta_dot / v) - theta_dot            
  #print(betadot)
  return betadot 

def betadot_slow(deltadot,delta):
  P = Parameters()
  betadot_sl = (P.lr * deltadot) / (P.L*np.cos(delta)**2 * (1 + (np.tan(delta)**2 * P.lr/P.L)**2))
  return betadot_sl 

def thetacc_Fast(v,a_l,theta_dot,betadot,delta):
  P = Parameters()    
  thetacc= P.mu*P.m/(v*P.I*(P.lr+P.lf))*(P.lf**2*P.C_Sf*(P.g*P.lr-a_l*P.h) + P.lr**2*P.C_Sr*(P.g*P.lf + a_l*P.h))*theta_dot \
                +P.mu*P.m/(P.I*(P.lr+P.lf))*(P.lr*P.C_Sr*(P.g*P.lf + a_l*P.h) - P.lf*P.C_Sf*(P.g*P.lr - a_l*P.h))*betadot \
                +P.mu*P.m/(P.I*(P.lr+P.lf))*P.lf*P.C_Sf*(P.g*P.lr - a_l*P.h)*delta
  return thetacc

def thetacc_slow(a_l,beta,delta,betadot,theta_dot,deltadot,yc):
  P = Parameters()
  thetacc_sl = 1/P.L*(a_l*np.cos(beta)*np.tan(delta)-v*np.sin(beta)*np.tan(delta)*betadot+ delta*np.cos(theta_dot)*deltadot/np.cos(yc)**2)
  return thetacc_sl

def xcdot(v,theta,beta):             
  xc_dot_global = v * np.cos(theta + beta)
  return xc_dot_global

def ycdot(v,theta,beta): 
  yc_dot_global = v * np.sin(theta + beta)  
  return yc_dot_global

def xcdotveh(v,beta):
    xc_dot = v*np.cos(beta)
    return xc_dot

def ycdotveh(v,beta):
    yc_dot = v*np.sin(beta)
    return yc_dot
    
def xcdotdot(a_l,theta,beta,theta_dot,betadot,v):
  xc_dot_dot=(a_l*np.cos(theta)) #- (v * np.sin(theta)*theta_dot) - (v * np.sin(theta)*betadot)
  return xc_dot_dot

def ycdotdot(a_l,theta,beta,theta_dot,betadot,v):
  yc_dot_dot = (a_l*np.sin(theta))#- (np.cos(theta))- (np.cos(theta))
  rospy.loginfo(yc_dot_dot)
  return yc_dot_dot



def publish():
    dyn_msg = lat_msgs()
    dyn_msg.xc = 0
    dyn_msg.yc= 0
    dyn_msg.theta= 0
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

  

def dyn(v,a_l,xc,yc,xc_dot,yc_dot,theta,beta,delta,deltadot,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev):
    P=Parameters()
    steer_prev = 0
    steer_wheel = (7E-14 * Steering_Input**2) + (10.99 * Steering_Input) - 3E-14
    delta = delta_Fast(Steering_Input,steer_wheel)
    steer_prev = delta

    deltadot = deltadot_Fast(delta,steer_prev)

    if deltadot > 0:
        deltadot = min(deltadot, P.deltadot_max)
    else:
        deltadot = max(deltadot, -P.deltadot_max)
    if delta > 0 :

        delta= min(delta,P.delta_max)

    else:

        delta= max(delta,P.delta_min)

    steer_prev = delta

    if v >= 0.1:
        #implementing the differential equations  
        beta = beta_Fast(delta)
        
        theta_dot = theta_dot_Fast(v,beta,delta)
        betadot = betadot_Fast(v,a_l,delta,beta,theta_dot)

        #beta+=betadot*P.sample_time
        theta +=  theta_dot * P.sample_time  


        xc_dot_global = xcdot(v,theta,beta)
        yc_dot_global = ycdot(v,theta,beta)

        xc+= xc_dot_global * P.sample_time                      
        yc+= yc_dot_global * P.sample_time 

        xc_dot = xcdotveh(v,beta)
        yc_dot = ycdotveh(v,beta)

        xc_dot_dot=xcdotdot(a_l,theta,beta,theta_dot,betadot,v) 
        yc_dot_dot=ycdotdot(a_l,theta,beta,theta_dot,betadot,v)

        steer_prev = delta

        return xc,yc,xc_dot,yc_dot,theta,beta,delta,deltadot,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev

    else:
        beta = beta_Fast(delta)
        theta_dot = theta_dot_Fast(v,beta,delta)
        betadot = betadot_slow(deltadot,delta)


        #beta+=betadot*P.sample_time 

        theta = theta_dot * P.sample_time 

  
        xc_dot_global = xcdot(v,theta,beta)
        yc_dot_global = ycdot(v,theta,beta)

        xc+= xc_dot_global * P.sample_time                      
        yc+= yc_dot_global * P.sample_time 

        xc_dot = xcdotveh(v,beta)
        yc_dot = ycdotveh(v,beta)

        xc_dot_dot=xcdotdot(a_l,theta,beta,theta_dot,betadot,v) 
        yc_dot_dot=ycdotdot(a_l,theta,beta,theta_dot,betadot,v)

        steer_prev = delta
        
        return xc,yc,xc_dot,yc_dot,theta,beta,delta,deltadot,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev



def Subscriber():
    #global steer_prev
    #steer_prev = 0 
    #P = Parameters()
    rospy.Subscriber('dyn_dynamics', dyn_msgs,callback_function)
    rospy.Subscriber('alle_inputs', alle_msgs,callback_function_1)
    rospy.spin()

def callback_function(message):
    
    global xc,yc,xc_dot,yc_dot,theta,beta,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev,Steering_Input
    xc= message.xc
    yc= message.yc
    theta = message.theta
    beta = message.beta
    theta_dot = message.theta_dot
    betadot = message.betadot
    xc_dot = message.xc_dot
    yc_dot = message.yc_dot
    xc_dot_dot = message.xc_dot_dot
    yc_dot_dot = message.yc_dot_dot 
    thetacc = message.thetacc
    steer_prev = message.steer_prev

    
def callback_function_1(message):
    
    global v,a_l,Steering_Input
    P = Parameters()
    steer_prev = 0
    Steering_Input = message.Steering
    v = message.Velocity
    a_l = message.Acc
    steer_wheel = (7E-14 * Steering_Input**2) + (10.99 * Steering_Input) - 3E-14
    delta = steer_wheel / P.r
    deltadot= (delta-steer_prev) / P.sample_time
    steer_prev= delta
    beta = np.arctan(P.lr * np.tan(delta) / P.L)
    xc_1,yc_1,xc_dot_1,yc_dot_1,theta_1,beta_1,delta_1,deltadot_1,theta_dot_1,betadot_1,xc_dot_dot_1,yc_dot_dot_1,thetacc_1,steer_prev_1 =dyn(v,a_l,xc,yc,xc_dot,yc_dot,theta,beta,delta,deltadot,theta_dot,betadot,xc_dot_dot,yc_dot_dot,thetacc,steer_prev)

    lat_msg = lat_msgs()
    lat_msg.xc = xc_1
    lat_msg.yc = yc_1
    lat_msg.theta = theta_1
    lat_msg.theta_dot =theta_dot_1
    lat_msg.xc_dot = xc_dot_1
    lat_msg.yc_dot = yc_dot_1
    lat_msg.beta = beta_1
    lat_msg.betadot = betadot_1
    lat_msg.xc_dot_dot = xc_dot_dot_1
    lat_msg.yc_dot_dot = yc_dot_dot_1
    lat_msg.thetacc = thetacc_1
    lat_msg.steer_prev = steer_prev_1


    rospy.loginfo(lat_msg)
    publisher.publish(lat_msg)
    

        
    
if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)
        
    rospy.init_node("Te")
    publisher = rospy.Publisher('lateral_dynamics', lat_msgs,queue_size=10)
    for i in range(500):
        publish()
    Subscriber()
    

if os.name != 'nt':
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
