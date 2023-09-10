#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

posx = 0
posy = 0 
pitch = 0
roll = 0
yaw = 0
vel = 0

vel_msg = Twist()

rospy.init_node("Autonomous_Systems_MS_2_OLR_Team_4" , anonymous= True)

vel_pub = rospy.Publisher("ackermann_steering_controller/cmd_vel", Twist , queue_size = 10)

def callback(data): 
    global posx , posy , roll, pitch, yaw, vel 
    print("----------------------------------------------------------------------------------")
    pos = data.pose.pose.position
    ori = data.pose.pose.orientation
    ori_list = [ori.x, ori.y, ori.z, ori.w]
    posx = round(pos.x,4)
    posy = round(pos.y,4)
    (roll, pitch, yaw) = euler_from_quaternion (ori_list)
    vel = round(data.twist.twist.linear.x,4)

vel_sub = rospy.Subscriber("ackermann_steering_controller/odom", Odometry, callback)

while not rospy.is_shutdown(): 
    
    vel_msg.linear.x = rospy.get_param("~Vel_OL")
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
   
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = rospy.get_param("~Ang_OL")

  
    print("x="+ str(posx) + "  y="+ str(posy) + "  roll="+ str(roll) + "  pitch="+ str(pitch) + "  yaw="+ str(yaw) + "  vel="+ str(vel))

    vel_pub.publish(vel_msg)
    
    pass