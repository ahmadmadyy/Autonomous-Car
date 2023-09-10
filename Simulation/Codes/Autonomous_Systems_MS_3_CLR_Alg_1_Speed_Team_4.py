#!/usr/bin/env python3

import rospy
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

posx = 0
posy = 0 
pitch = 0
roll = 0
yaw = 0
vel = 0

vel_msg = Twist()

rospy.init_node("Autonomous_Systems_MS_3_CLR_Alg_1_Speed_Team_4" , anonymous= True)

V_des = rospy.get_param("~V_desired")

vel_pub = rospy.Publisher("ackermann_steering_controller/cmd_vel", Twist , queue_size = 10)

def callback(data):
    global posx , posy , roll, pitch, yaw, vel
    print("----------------------------------------------------------------------------------")
    ori = data.pose[1].orientation
    ori_list = [ori.x, ori.y, ori.z, ori.w]
    posx = round(data.pose[1].position.x,4)
    posy = round(data.pose[1].position.y,4)
    (roll, pitch, yaw) = euler_from_quaternion (ori_list)
    vel = round(data.twist[1].linear.x,4)

vel_sub = rospy.Subscriber("gazebo/model_states", ModelStates, callback)


def apply_PIDv(velocity, targetVelocity):
    """Apply the P-Controller."""
    Kp = 80
    diff = targetVelocity - velocity
    # compute speed
    speed = Kp * diff

    return speed


while not rospy.is_shutdown(): 
    
    vel_msg.linear.x = apply_PIDv(vel, V_des)
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
   
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

  
    print("x="+ str(posx) + "  y="+ str(posy) + "  roll="+ str(round(roll,4)) + "  pitch="+ str(round(pitch,4)) + "  yaw="+ str(round(yaw,4)) + "  vel="+ str(vel))

    vel_pub.publish(vel_msg)
    
    
    pass