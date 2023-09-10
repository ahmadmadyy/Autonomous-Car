#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, tty, termios

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

posx = 0
posy = 0 
pitch = 0
roll = 0
yaw = 0
vel = 0

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

def callback(data): 
    global posx , posy , roll, pitch, yaw, vel 
    pos = data.pose.pose
    ori = data.pose.pose.orientation
    ori_list = [ori.x, ori.y, ori.z, ori.w]
    posx = round(pos.position.x,4)
    posy = round(pos.position.y,4)
    (roll, pitch, yaw) = euler_from_quaternion (ori_list)
    vel = round(data.twist.twist.linear.x,4)

vel_sub = rospy.Subscriber("ackermann_steering_controller/odom", Odometry, callback)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node("Autonomous_Systems_MS_2_Teleop_Team_4" , anonymous= True)
    pub = rospy.Publisher('ackermann_steering_controller/cmd_vel', Twist, queue_size=1)

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    x = 0
    th = 0
    status = 0

    try:
        print(vels(speed, turn))
        print("Use w,a,s,d to move the car, press q to quit")

        while(1):
            print("VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES")
            print("VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES")
            print("VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES")
            print("VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES VALUES")
            print("x="+ str(posx) + "  y="+ str(posy) + "  roll="+ str(roll) + "  pitch="+ str(pitch) + "  yaw="+ str(yaw) + "  vel="+ str(vel))
            key = getKey()
            if key == 'q':
                break

            if key == 'w':
                x = speed
                th = 0
            elif key == 's':
                x = -speed
                th = 0
            elif key == 'd':
                x = 0
                th = -turn
            elif key == 'a':
                x = 0
                th = turn
            else:
                x = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = x
            twist.linear.y = 0
            twist.linear.z = 0

            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = th

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0

        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0

        pub.publish(twist)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


    