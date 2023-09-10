#!/usr/bin/env python3

import rospy
import math
import time
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

steer = 0
x_des = 20
start_time = time.time()

rospy.init_node("combine" , anonymous= True)

vel_msg = Twist()
pose1 = Pose()

lane_des = rospy.get_param("~lane_desired")
V_des = rospy.get_param("~V_desired")

vel_pub = rospy.Publisher("ackermann_steering_controller/cmd_vel", Twist , queue_size = 10)


def apply_PIDv(velocity, targetVelocity):
    global flag
    """Apply the P-Controller."""
    Kp = 80
    diff = targetVelocity - velocity
    # compute speed
    speed = Kp * diff

    return speed

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
        posx = pos.x
        posy = pos.y
        (roll, pitch, yaw) = euler_from_quaternion (ori_list)
        vel = round(data.twist[1].linear.x,4)


vel_sub = rospy.Subscriber("gazebo/model_states", ModelStates, callback)

flag = False

time.sleep(5)

while not rospy.is_shutdown():
    
    if flag == False:
        vel_msg.linear.x = apply_PIDv(vel, V_des)
    if posx <= x_des + 0.3 and posx >= x_des - 0.3:
        flag = True
    if flag:
        vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0

    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = stanleyControl(lane_des)

    if round(yaw,4) <= 0.0002 and round(yaw,4) >= -0.0002:
        end_time = time.time()
        print("Time to reach the desired lane: " + str(end_time - start_time))
    print("x="+ str(posx) + "  y="+ str(posy) + "  roll="+ str(round(roll,4)) + "  pitch="+ str(round(pitch,4)) + "  yaw="+ str(yaw) + "  vel="+ str(vel))

    vel_pub.publish(vel_msg)
