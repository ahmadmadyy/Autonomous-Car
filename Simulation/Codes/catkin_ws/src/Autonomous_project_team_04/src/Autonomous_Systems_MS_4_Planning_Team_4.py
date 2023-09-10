#!/usr/bin/env python

#########################################################################################################
#Import the required libraries:
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose,Twist
from gazebo_msgs.msg import ModelStates
import numpy as np
import math
import time

time.sleep(5)
#########################################################################################################

#########################################################################################################
#######################################################################
#Initialize ROS Node
rospy.init_node('Autonomous_Systems_MS_4_Planning_Team_4') #Identify ROS Node
#######################################################################

#######################################################################
#ROS Publisher Code for Steering
pub1 = rospy.Publisher('/Longitudinal_Driving_Velocity', Float64, queue_size=10)
pub2 = rospy.Publisher('/Lateral_Distance', Float64, queue_size=10)
rate = rospy.Rate(10) # rate of publishing msg 10hz
#######################################################################

#######################################################################
#ROS Subscriber Variables
Actual_position = Pose()
flag_Sp = 0
flag_Pos = 0
#######################################################################

#######################################################################
#ROS Subscriber Code for Position and Velocity
def callback_Act_Pos(data):
  global Actual_position, flag_Pos
 
  if flag_Pos == 0:
    Actual_position = data
    flag_Pos = 1

sub1 = rospy.Subscriber('/steer_bot/gazebo/KalmanValP', Pose, callback_Act_Pos)

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
V_Des = 0.8
Lane_Des = 0.5
#######################################################################
#Simulation While Loop
#######################################################################
while not rospy.is_shutdown():
  if flag_Pos == 1:
    
    if Actual_position.position.x < 2.8 and Actual_position.position.x > 2.3:
      V_Des = 0.5
      Lane_Des = -0.5
    if Actual_position.position.x >= 2.8 and Actual_position.position.x <= 5.5:
      V_Des = 0.8
    if Actual_position.position.x > 4 and Actual_position.position.x < 5.5:
      Lane_Des = -0.5
    if Actual_position.position.x < 6.8 and Actual_position.position.x > 6.3:
      V_Des = 0.5
      Lane_Des = 0.5
    if Actual_position.position.x > 7.5:
      V_Des = 0.8
      Lane_Des = 0.5
    if Actual_position.position.x > 10:
      V_Des = 0

    flag_Sp = 0
    flag_Pos = 0

  pub1.publish(V_Des)	#Publish msg
  pub2.publish(Lane_Des)	#Publish msg
  rate.sleep()		#Sleep with rate
#######################################################################
#########################################################################################################



