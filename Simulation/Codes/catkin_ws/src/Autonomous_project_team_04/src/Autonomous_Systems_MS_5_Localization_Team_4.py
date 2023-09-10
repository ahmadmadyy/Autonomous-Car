#!/usr/bin/env python3

import rospy #library to connect ROS with python
from matplotlib import pyplot as plt
import numpy as np #library used for mathematical operations
import math
import random

from gazebo_msgs.msg import ModelStates #import msg data type "Odometry" from nav_msgs dependency to be subscribed 
from geometry_msgs.msg import Twist, Pose #import msg data type "Twist" and "Pose" from geometry_msgs dependency to be published and subscribed


rospy.init_node('Autonomous_Systems_MS_5_Localization_Team_4', anonymous = True) #Identify Ros Node

pub1 = rospy.Publisher('gazebo/KalmanValP', Pose, queue_size=10)
pub2 = rospy.Publisher('gazebo/KalmanValT', Twist, queue_size=10)
Control_InputP = Pose()
Control_InputT = Twist()
rate = rospy.Rate(10) # rate of publishing msg 10hz

## Method used to transform from Euler coordinates to Quaternion coordinates
def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]

####################################################################################################################################################################
## Method used to transform from Quaternion coordinates to Euler coordinates
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

##Create arrays for printing
XModel = [] #Array to hold value of x obtained by the sensor
YModel = [] #Array to hold value of y obtained by the sensor
ThetaModel = [] #Array to hold value of theta obtained by the sensor

XNoisy = [] #Array to hold noisy value of x coordinates
YNoisy = [] #Array to hold noisy value of y coordinates
ThetaNoisy = [] #Array to hold noisy value of theta coordinates

XFiltered = [] #Array to hold Filtered value of x coordinates
YFiltered = [] #Array to hold Filtered value of y coordinates
ThetaFiltered = [] #Array to hold Filtered value of theta coordinates

position_noisy = np.zeros((1,6)) #Create empty array

def noisy_add(position):
    #Define the global variables represented as the standard deviation of each state
    global position_noisy

    xnoise = position[0] + random.gauss(0,0.01)			##Noisy data calculated at x
    ynoise = position[1] + random.gauss(0,0.01)			##Noisy data calculated at y
    thetanoise = position[3] + random.gauss(0,0.01)		##Noisy data calculated at z

    position_noisy = [xnoise, ynoise, position[2], thetanoise, position[4], position[5]]##Store the noisy position in array))
    return(position_noisy)			#Return the noisy position


## Prediction stage in Kalman filter
def kf_prediction(Xprev,Pprev, A, Q, B, U):
    Xpredicted = np.matmul(A, Xprev) + np.dot(B, U)			##Predicted state vector			
    Ppredicted = np.matmul(A, np.matmul(Pprev, np.transpose(A))) + Q	##Predicted error co-variance matrix	
    return (Xpredicted, Ppredicted)
####################################################################################################################################################################

####################################################################################################################################################################
## Correction stage in Kalman filter
def kf_correction(Xpredicted, Ppredicted, C, Z, R):			
    CTrans = np.transpose(C)				
    num = np.matmul(Ppredicted, CTrans)		##Numerature of Kalman gain equation
    den1 = np.matmul(C, Ppredicted) 		##CMatrix * PMatrix
    den2 = np.matmul(den1, CTrans) + R  	##CMatrix * PMatrix * CMatrix^T _+ R
    den = np.matrix(den2)  			##Place denominator in matrix form  
    deninv = den.getI()				##(CMatrix * PMatrix * CMatrix^T _+ R) Inverse 	
    KGain = np.matmul(num, deninv) 		##Calculate the Kalman gain

    Xfiltered = Xpredicted + np.matmul(KGain, (Z - np.matmul(C, Xpredicted))) 	##Estimated updated state vector
    Pfiltered = Ppredicted - np.matmul(KGain, np.matmul(C, Ppredicted)) 	##Estimated updated error co-variance matrix
    return (Xfiltered, Pfiltered)
####################################################################################################################################################################

#ROS Subscriber Code for Position
flag_initial = 0			#Initialize flag by zero

def callback_Init(data):
    global flag_initial
    
    if data.name.__contains__("steer_bot"):
        flag_initial = 1
        sub1.unregister()

sub1 = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback_Init)

#ROS Subscriber Code for Position
flag_cont = 0			#Initialize flag by zero
pos_msg = Pose()		#Identify msg variable of data type Pose
position = np.zeros((1,6))	#Identify array of six elements all initialized by zero
Velocity_msg = Twist()		#Identify msg variable of data type Twist
Time = 0


def callback(data):
    global pos_msg, flag_cont, position, Velocity_msg   #Identify msg variable created as global variable
    msg = data ##Extract the data sent as message
    
    if data.name.__contains__("steer_bot"):
        pos_msg.position.x = round(msg.pose[2].position.x, 4)		#Round the value of x to 4 decimal places
        pos_msg.position.y = round(msg.pose[2].position.y, 4)		#Round the value of y to 4 decimal places
        pos_msg.position.z = round(msg.pose[2].position.z, 4)		#Round the value of y to 4 decimal places
        pos_msg.orientation.x = round(msg.pose[2].orientation.x, 4)		#Round the value of theta to 4 decimal places
        pos_msg.orientation.y = round(msg.pose[2].orientation.y, 4)		#Round the value of theta to 4 decimal places
        pos_msg.orientation.z = round(msg.pose[2].orientation.z, 4)		#Round the value of theta to 4 decimal places
        pos_msg.orientation.w = round(msg.pose[2].orientation.w, 4)		#Round the value of theta to 4 decimal places
        [yaw, pitch, roll] = quaternion_to_euler(pos_msg.orientation.x, pos_msg.orientation.y, pos_msg.orientation.z, pos_msg.orientation.w) #Transform from Quaternion to Euler coordinates
        position = [pos_msg.position.x,pos_msg.position.y,pos_msg.position.z,yaw, pitch, roll] ##Store the position in array

        Velocity_msg.linear.x = round(msg.twist[2].linear.x, 4)		#Round the value of x to 4 decimal places
        Velocity_msg.angular.z = round(msg.twist[2].angular.z, 4)		#Round the value of z to 4 decimal places
        flag_cont = 1

sub2 = rospy.Subscriber('/steer_bot/gazebo/model_states', ModelStates, callback)

####################################################################################################################################################################
##Initialization:
P = [[1,0,0],[0,1,0],[0,0,1]]		##Error co-variance matrix P (3x3)
Q = [[0.01,0,0],[0,0.01,0],[0,0,0.01]]		##Process noise matrix Q (3x3)
R = 0.1						##Measurement noise matrix R (1x1)

U = [[Velocity_msg.linear.x],[Velocity_msg.angular.z]]					##Control input matrix U (2x1)
X = [[0],[0],[0]]				##Initial state X (3x1)

A = [[1,0,0],[0,1,0],[0,0,1]]			##State transition matrix A (3x3)
B = [[0.1,0],[0,0],[0,0]]			##Input matrix B (3x2)
C = [[1,0,0]]			    	##Measurement matrix C (1x3)

iterations = 250
i = 0

while not rospy.is_shutdown():
    if i < iterations:
        if flag_initial == 1:
            ##Get the initial states
            x_p = 0
            y_p = 0
            theta_p = 0
            X = [[x_p],[y_p],[theta_p]] 	##Set the states of the system
            Z = 0			##Set the sensor reading for the x position of the robot
            (Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
            (Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Z , R)	##Get the corrected states
            (Ypredicted, Pypredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
            (Yfiltered, Pyfiltered) = kf_correction(Ypredicted, Pypredicted, C, Z , R)
            (Tpredicted, Ptpredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
            (Tfiltered, Ptfiltered) = kf_correction(Tpredicted, Ptpredicted, C, Z , R)
            Control_InputP.position.x = Xfiltered[0]
            Control_InputP.position.y = Yfiltered[0]
            Control_InputP.position.z = 0
            [Control_InputP.orientation.x,Control_InputP.orientation.y,Control_InputP.orientation.z,Control_InputP.orientation.w] = euler_to_quaternion(Tfiltered[0], 0, 0)
            pub1.publish(Control_InputP) #publish the position

            Control_InputT.linear.x = Velocity_msg.linear.x
            Control_InputT.linear.y = 0
            Control_InputT.linear.z = 0
            Control_InputT.angular.x = 0
            Control_InputT.angular.y = 0
            Control_InputT.angular.z = Velocity_msg.angular.z
            pub2.publish(Control_InputT) #publish the speed

            XModel.append(Xpredicted[0]) 	#Array to hold value of x obtained by the sensor
            YModel.append(Xpredicted[0]) 	#Array to hold value of y obtained by the sensor
            ThetaModel.append(Xpredicted[0]) 	#Array to hold value of theta obtained by the sensor
            XNoisy.append(float(x_p)) 			#Array to hold noisy value of x coordinates
            YNoisy.append(float(y_p)) 			#Array to hold noisy value of y coordinates
            ThetaNoisy.append(float(theta_p))		#Array to hold noisy value of theta coordinates
            XFiltered.append(float(Xfiltered[0])) 	#Array to hold Filtered value of x coordinates
            YFiltered.append(float(Xfiltered[0]))	#Array to hold Filtered value of y coordinates
            ThetaFiltered.append(float(Xfiltered[0])) #Array to hold Filtered value of theta coordinates
            flag_initial =  0

        if flag_cont == 1:
            position_noise = noisy_add(position)
            ##filtering error in X
            x_p = position[0]
            Zx = x_p		##Set the sensor reading for the x position of the robot
            X = Xfiltered	##Update the states with the filtered states
            P = Pfiltered	##Update the error co-variance matrix
            (Xpredicted, Ppredicted) = kf_prediction(X, P, A, Q, B, U)			##Get the predicted states
            (Xfiltered, Pfiltered) = kf_correction(Xpredicted, Ppredicted, C, Zx, R)		##Get the corrected states
            ##filtering error in Y
            y_p = position[1]
            Zy = y_p
            Y = Yfiltered	##Update the states with the filtered states
            Py = Pyfiltered	##Update the error co-variance matrix
            (Ypredicted, Pypredicted) = kf_prediction(Y, Py, A, Q, B, U)			##Get the predicted states
            (Yfiltered, Pyfiltered) = kf_correction(Ypredicted, Pypredicted, C, Zy, R)		##Get the corrected states
            ##filtering error in Theta
            theta_p = position[3]
            Zt = theta_p
            T = Tfiltered	##Update the states with the filtered states
            Pt = Ptfiltered	##Update the error co-variance matrix
            (Tpredicted, Ptpredicted) = kf_prediction(T, Pt, A, Q, B, U)			##Get the predicted states
            (Tfiltered, Ptfiltered) = kf_correction(Tpredicted, Ptpredicted, C, Zt, R)		##Get the corrected states
            Control_InputP.position.x = Xfiltered[0]
            Control_InputP.position.y = Yfiltered[0]
            Control_InputP.position.z = 0
            [Control_InputP.orientation.x,Control_InputP.orientation.y,Control_InputP.orientation.z,Control_InputP.orientation.w] = euler_to_quaternion(Tfiltered[0], 0, 0)
            pub1.publish(Control_InputP) #publish the position

            Control_InputT.linear.x = Velocity_msg.linear.x
            Control_InputT.linear.y = 0
            Control_InputT.linear.z = 0
            Control_InputT.angular.x = 0
            Control_InputT.angular.y = 0
            Control_InputT.angular.z = Velocity_msg.angular.z
            pub2.publish(Control_InputT) #publish the speed

            print("Xfiltered = " + str(Xfiltered[0]))
            print("Yfiltered = " + str(Yfiltered[0]))
            print("Tfiltered = " + str(Tfiltered[0]))
            #Fill arrays for plots
            XModel.append(Xpredicted[0]) 	#Array to hold value of x obtained by the sensor
            YModel.append(Ypredicted[0]) 	#Array to hold value of y obtained by the sensor
            ThetaModel.append(Tpredicted[0]) 	#Array to hold value of theta obtained by the sensor
            XNoisy.append(float(x_p)) 			#Array to hold noisy value of x coordinates
            YNoisy.append(float(y_p)) 			#Array to hold noisy value of y coordinates
            ThetaNoisy.append(float(theta_p))		#Array to hold noisy value of theta coordinates
            XFiltered.append(float(Xfiltered[0])) 	#Array to hold Filtered value of x coordinates
            YFiltered.append(float(Yfiltered[0]))	#Array to hold Filtered value of y coordinates
            ThetaFiltered.append(float(Tfiltered[0])) #Array to hold Filtered value of theta coordinates
            flag_cont = 0

        rate.sleep()
        i = i + 1
    else:
        ##Plotting of signals from sensor and noisy signals
        plt.figure(1)
        line_1 = plt.plot(XModel, 'r-', label='X-Model')
        line_2 = plt.plot(XNoisy, 'b-', label='X-Noisy')
        line_3 = plt.plot(XFiltered, 'g-', label='X-Filtered')
        plt.legend()

        plt.figure(2)
        line_1 = plt.plot(YModel, 'r-', label='Y-Model')
        line_2 = plt.plot(YNoisy, 'b-', label='Y-Noisy')
        line_3 = plt.plot(YFiltered, 'g-', label='Y-Filtered')
        plt.legend()

        plt.figure(3)
        line_1 = plt.plot(ThetaModel, 'r-', label='Theta-Model')
        line_2 = plt.plot(ThetaNoisy, 'b-', label='Theta-Noisy')
        line_3 = plt.plot(ThetaFiltered, 'g-', label='Theta-Filtered')
        plt.legend()
        plt.show()


