#!/usr/bin/env python3
import RPi.GPIO as GPIO          
from time import sleep
import signal
import time
import sys
import RPi.GPIO as GPIO
import numpy as np
import rospy
from std_msgs.msg import Float32
from gpiozero import Servo
import smbus#import SMBus module of I2C
import math
import numpy as np
from gpiozero.pins.pigpio import PiGPIOFactory

factory = PiGPIOFactory()
servo = Servo(17,pin_factory=factory)
rospy.init_node('lateral')
#some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47
def MPU_Init():
    #write to sample rate register
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)
def read_raw_data(addr):
    #Accelero and Gyro value are 16-bit
        high = bus.read_byte_data(Device_Address, addr)
        low = bus.read_byte_data(Device_Address, addr+1)
        value = ((high << 8) | low)
        if(value > 32768):
                value = value - 65536
        return value
bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address
MPU_Init()
average=0
Distance=0
x_global=0
y_global=0
yaw_map=0
for x in range(100):
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Gz = gyro_z/131.0
    time.sleep(0.01)
    average=average+Gz
error_yaw_map= average/100
Lane_Des=0
yaw=0
Distance=0
timer=time.time()
servo.value=0.1
delta=0
average=0   
Actual_Speed = Float32()
Distance = Float32()
Lane_des = 0
def callback_Act_Sp(data):
    global Actual_Speed
    Actual_Speed = data.data
sub1 = rospy.Subscriber('/Driving_Velocity', Float32, callback_Act_Sp)

def callback_Distance(data):
    global Distance
    Distance = data.data
sub2 = rospy.Subscriber('/Distance', Float32, callback_Distance)

def callback_Des_Lane(data):
    global Lane_Des
    Lane_Des = data.data
#     print('Lane_Des = '+str(Lane_Des))
sub3 = rospy.Subscriber('/Lateral_Distance', Float32, callback_Des_Lane)

def Stanley_Control(Lane_Des,x,y,heading,V_Act):
    V_Act=round(V_Act,2)
    print("V_Act : "+str(V_Act))
    K_v = 110 
    x_Lane = x + 1*np.sign(V_Act)
    Lookahead_pnt = [x_Lane,Lane_Des]
    #print('Lookahead_pnt = '+str(Lookahead_pnt))
    dy = Lookahead_pnt[1]-(y +0.15*np.sin(heading))
    th_p = 0 - heading  
    Steer_Cont = th_p + (np.arctan(((K_v*dy)/(V_Act+0.0001)))/3)-0.1
#     print('Steer_Cont = '+str(Steer_Cont))      
    if(abs(Steer_Cont) >= 0.53):
        Steer_Cont = np.sign(Steer_Cont)*0.53
    
    return Steer_Cont
timer=time.time()
Distance=0
x_global=0
y_global=0
yaw_map=0
yaw=0
Actual_Speed=0.0001
yaw_map_old=0

rate = rospy.Rate(100)
while True:
    Actual_Speed=round(Actual_Speed,2)
    gyro_z = read_raw_data(GYRO_ZOUT_H)
    Gz = gyro_z/131.0
    Gz =Gz-error_yaw_map
    Gz=round(Gz,0)
    dt = time.time() - timer
    timer = time.time()
    dt= round(dt,2)
    
    yaw=yaw+Gz*dt
    yaw_map=(yaw - -11.2) * (90 - -90) / (11.2 - -11.2) -90
    yaw_map=round(yaw_map,2)
    x_global = Distance * np.cos(-np.deg2rad(yaw_map))
    x_global = round(x_global,2)
    y_global = y_global+(Actual_Speed * dt*np.sin(np.deg2rad(yaw_map)))
    y_global = round(y_global,3)
    #print("Actual_Speed :" + str(Actual_Speed))
    print("y_global :" + str(y_global))
    print("yaw_map: " + str(yaw_map))
#     print("Distance: " + str(Distance))
    delta=round(Stanley_Control(Lane_Des,x_global,y_global,np.deg2rad(yaw_map),Actual_Speed),2)
    #print("delta: " + str(delta))
#     print("Lane: " + str(Lane_Des))
    servo.value=delta
    #print("x_global: " + str(x_global) + "  y_global: " + str(y_global) +" \n x_r: " + str(Distance)+"  yaw_map " + str(round(yaw_map,1)))
    
    rate.sleep()
        

 
    

