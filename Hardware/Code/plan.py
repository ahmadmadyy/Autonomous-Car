#!/usr/bin/env python
    
import rospy
from std_msgs.msg import Float32
import numpy as np
import math
import time
#rate2 = rospy.Rate(10)
rospy.init_node('plan')
pub1 = rospy.Publisher('/Long_Velocity', Float32,queue_size=1)
pub2 = rospy.Publisher('/Lateral_Distance', Float32,queue_size=1)
Distance = Float32()
flag_Dist = 0

def callback_Dist(data):
    global Distance
    Distance = data.data
   
sub1 = rospy.Subscriber('/Distance', Float32, callback_Dist)
speed = 1
Lane_Des=0
t=10
rate = rospy.Rate(100)#1000
while True:
#     rate = rospy.Rate(t)
#   
#     if Distance>=0:
#         Lane_Des=0.3
#     if Distance>=1.5:
#         Lane_Des=-0.3
#     if Distance>=4.5:
#         Lane_Des=0.4    
    pub2.publish(Lane_Des)
    pub1.publish(speed)

    #print("\ntime Plan"+str(t))
    rate.sleep()
    
    
