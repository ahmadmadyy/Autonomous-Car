#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
from __future__ import print_function,division
import rospy
from geometry_msgs.msg import Pose,Twist
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64
import numpy as np
import math
import time
from matplotlib import pyplot as plt

time.sleep(5)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Control_Module_Lateral_Motion') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Steering', Float64, queue_size=10)
#pub2 = rospy.Publisher('gazebo/KalmanVal', ModelStates, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
Actual_Speed = Twist()
Lane_Des = 0
flag_Pos = 0
flag_Sp = 0
flag_Lat = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Act_Pos(data):
  global Actual_position, flag_Pos
 
  if flag_Pos == 0:
    Actual_position = data
    flag_Pos = 1

sub1 = rospy.Subscriber('/steer_bot/gazebo/KalmanValP', Pose, callback_Act_Pos)
#######################################################################

def callback_Act_Sp(data):
  global Actual_Speed, flag_Sp
 
  if flag_Sp == 0:
    Actual_Speed = data
    flag_Sp = 1

sub3 = rospy.Subscriber('/steer_bot/gazebo/KalmanValT', Twist, callback_Act_Sp)

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Des_Lane(data):
  global Lane_Des, flag_Lat
 
  if flag_Lat == 0:
    Lane_Des = data.data
    print('Lane_Des = '+str(Lane_Des))
    flag_Lat = 1
  
sub2 = rospy.Subscriber('/Lateral_Distance', Float64, callback_Des_Lane)
#######################################################################
#########################################################################################################

#########################################################################################################
def quaternion_to_euler(x, y, z, w):
  t0 = +2.0 * (w * x + y * z)
  t1 = +1.0 - 2.0 * (x * x + y * y)
  roll = math.atan2(t0, t1)
  t2 = +2.0 * (w * y - z * x)
  t2 = +1.0 if t2 > +1.0 else t2
  t2 = -1.0 if t2 < -1.0 else t2
  pitch = math.asin(t2)
  t3 = +2.0 * (w * z + x * y)
  t4 = +1.0 - 2.0 * (y * y + z * z)
  yaw = math.atan2(t3, t4)
  return [yaw, pitch, roll]
#########################################################################################################

#########################################################################################################
def Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  global Wheel_Base
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act)#K_p*V_Act
  
  Lookahead_pnt = [x_Lane,Lane_Des]
  dx = Lookahead_pnt[0]-Pos_Act[0]
  dy = Lookahead_pnt[1]-Pos_Act[1]
  L_d = np.sqrt((dx)**2 + (dy)**2)
  alpha = - Pos_Act[2] + np.arctan2(dy,dx)
  
  Steer_Cont = np.arctan(((2*Wheel_Base*np.sin(alpha))/(L_d)))
  
  if(abs(Steer_Cont) >= np.radians(30)):
    Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  print('Steer_Cont = '+str(Steer_Cont))
  return Steer_Cont
#########################################################################################################

#########################################################################################################
def Stanley_Control(Lane_Des,Pos_Act,V_Act):
  K_p = 0.5
  K_v = 1
  global Wheel_Base
  
  x_Lane = Pos_Act[0] + 1*np.sign(V_Act) #K_p*V_Act
  Lookahead_pnt = [x_Lane,Lane_Des]
  dx = Lookahead_pnt[0]-(Pos_Act[0]+Wheel_Base*np.cos(Pos_Act[2]))
  dy = Lookahead_pnt[1]-(Pos_Act[1]+Wheel_Base*np.sin(Pos_Act[2]))
  L_d = np.sqrt((dx)**2 + (dy)**2)
  
  th_p = 0 - Pos_Act[2] 
  try:
    Steer_Cont = th_p + np.arctan(((K_v*dy)/(V_Act)))
  except:
    Steer_Cont = 0
  if(abs(Steer_Cont) >= np.radians(30)):
    Steer_Cont = np.sign(Steer_Cont)*np.radians(30)
  return Steer_Cont
#########################################################################################################

Act_Lane = []
Desired_Lane = []

#########################################################################################################
Lateral_Controller = rospy.get_param("~Lateral_Controller")
Wheel_Base = rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 
#######################################################################
#Simulation While Loop
Steer_Cont = 0
iterations = 250
i = 0
#######################################################################
while not rospy.is_shutdown():
  if i < iterations:
    if flag_Pos == 1 and flag_Lat == 1 and flag_Sp == 1:
      V_Act = Actual_Speed.linear.x
      #print('Vel_Act = '+str(V_Act))

      Angles_Act = quaternion_to_euler(Actual_position.orientation.x, Actual_position.orientation.y, Actual_position.orientation.z, Actual_position.orientation.w)
      Yaw_act = Angles_Act[0] 
      Pos_Act = [Actual_position.position.x, Actual_position.position.y,Yaw_act]
      #print('Pos_Act = '+str(Pos_Act))

      if Lateral_Controller in "Pure_Pursuit":
        Steer_Cont = Pure_Pursuit_Control(Lane_Des,Pos_Act,V_Act)
      elif Lateral_Controller in "Stanley":
        Steer_Cont = Stanley_Control(Lane_Des,Pos_Act,V_Act)
      elif Lane_Des == Actual_position.position.y:
        Steer_Cont = 0

      flag_Pos = 0 
      flag_Lat = 0
      flag_Sp = 0

      Act_Lane.append(Actual_position.position.y)
      Desired_Lane.append(Lane_Des)
    
    pub1.publish(Steer_Cont)	#Publish msg
    rate.sleep()		#Sleep with rate
    i = i + 1
  else:
    ##Plotting of signals from sensor and noisy signals
    plt.figure(4)
    line_1 = plt.plot(Act_Lane, 'b-', label='Actual Lane')
    line_2 = plt.plot(Desired_Lane, 'r-', label='Desired Lane')
    plt.legend()
    plt.show()
#######################################################################
#########################################################################################################
