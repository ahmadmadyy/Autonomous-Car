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
rospy.init_node('Control_Module_Longitudinal_Motion') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Control_Action_Driving_Velocity', Float64, queue_size=10)
#pub2 = rospy.Publisher('gazebo/KalmanVal', ModelStates, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
Actual_Speed = Twist()
V_Des = 0
flag_Vel = 0
flag_Pos = 0
flag_Sp = 0
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
  global Actual_Speed
  global flag_Sp		#Identify a subscriber as global variable
 
  if flag_Sp == 0:
    Actual_Speed = data
    flag_Sp = 1

sub3 = rospy.Subscriber('/steer_bot/gazebo/KalmanValT', Twist, callback_Act_Sp)

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Des_Vel(data):
  global V_Des, flag_Vel
 
  if flag_Vel == 0:
    V_Des = data.data
    print('V_Des = '+str(V_Des))
    flag_Vel = 1

sub2 = rospy.Subscriber('/Longitudinal_Driving_Velocity', Float64, callback_Des_Vel)
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
def Speed_Control(V_Des,V_Act,tau):
  K_p = 2
  u = K_p*(V_Des-V_Act)
  if(np.abs(u) > 2):
    u = 2*np.sign(u)
  V_Cont = V_Act + tau*u
  return V_Cont
#########################################################################################################
Act_Speed = []
Des_Speed = []
#########################################################################################################
Wheel_Base = rospy.get_param("~Wheel_Base") #Check Gazebo Model Dimenssions 
#######################################################################
#Simulation While Loop
st_time = time.time()
tau = 0.01
V_Cont = 0
iterations = 250
i = 0
#######################################################################
while not rospy.is_shutdown():
  if i < iterations:
    st_time = time.time()
    if flag_Sp == 1 and flag_Vel == 1 and flag_Pos == 1:
      V_Act = Actual_Speed.linear.x
      print('Vel_Act = '+str(V_Act))
 
      Angles_Act = quaternion_to_euler(Actual_position.orientation.x, Actual_position.orientation.y, Actual_position.orientation.z, Actual_position.orientation.w)
      Yaw_act = Angles_Act[0]
      Pos_Act = [Actual_position.position.x, Actual_position.position.y,Yaw_act]
      print('Pos_Act = '+str(Pos_Act))
 
      V_Cont = Speed_Control(V_Des,V_Act,tau)
  
      flag_Sp = 0
      flag_Pos = 0
      flag_Vel = 0
      Act_Speed.append(V_Act)
      Des_Speed.append(V_Des)
  
    pub1.publish(V_Cont)	#Publish msg
    rate.sleep()		#Sleep with rate
    end_time = time.time()
    tau  = end_time-st_time
    i = i + 1
  else:
    plt.figure(5)
    line_1 = plt.plot(Act_Speed, 'b-', label='Actual Speed')
    line_2 = plt.plot(Des_Speed, 'r-', label='Desired Speed')
    plt.legend()
    plt.show()
#######################################################################
#########################################################################################################



