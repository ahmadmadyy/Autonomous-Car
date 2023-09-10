#!/usr/bin/env python3

import rospy
import math
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelStates

posx = 0
posy = 0 
pitch = 0
roll = 0
yaw = 0
vel = 0
x_des = posx + 10
vel_msg = Twist()
pose1 = Pose()

rospy.init_node("Autonomous_Systems_MS_3_CLR_Alg_2_Lateral_Team_4" , anonymous= True)

lane = rospy.get_param("~lane_desired")
steer = 0
vel_pub = rospy.Publisher("ackermann_steering_controller/cmd_vel", Twist , queue_size = 10)


def stanleyControl(l_des):
    global steer

    deltaL = l_des - posy
    thetaD = math.atan(deltaL)
    steer = thetaD - yaw

    if steer >= 0.5:
        steer = 0.5
    if steer <= -0.5:
        steer = -0.5
    return steer

def callback(data):
    global posx , posy , roll, pitch, yaw, vel
    print("----------------------------------------------------------------------------------")
    if data.name.__contains__("steer_bot"):
        pose1 = data.pose[1]
        pos = pose1.position
        ori = pose1.orientation
        ori_list = [ori.x, ori.y, ori.z, ori.w]
        posx = round(pos.x,4)
        posy = round(pos.y,4)
        (roll, pitch, yaw) = euler_from_quaternion (ori_list)
        vel = round(data.twist[1].linear.x,4)

flag = False

vel_sub = rospy.Subscriber("gazebo/model_states", ModelStates, callback)

while not rospy.is_shutdown():
    
    if flag == False:
        vel_msg.linear.x = 1 
    if posx <= x_des + 0.005 and posx >= x_des - 0.005:
        flag = True
    if flag:
        vel_msg.linear.x = 0    
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = stanleyControl(lane)


    print("x="+ str(posx) + "  y="+ str(posy) + "  roll="+ str(round(roll,4)) + "  pitch="+ str(round(pitch,4)) + "  yaw="+ str(round(yaw,4)) + "  vel="+ str(vel))

    vel_pub.publish(vel_msg)
    

