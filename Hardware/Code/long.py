#!/usr/bin/env python
import RPi.GPIO as GPIO          
from time import sleep
import signal
import time
import sys
import RPi.GPIO as GPIO
import numpy as np
import rospy
from std_msgs.msg import Float32
import smbus#import SMBus module of I2C
import math
#rate2 = rospy.Rate(10)
rospy.init_node('long')
pub1 = rospy.Publisher('/Driving_Velocity', Float32,queue_size=1)
pub2 = rospy.Publisher('/Distance', Float32,queue_size=1)
timer = time.time()
yaw=0
BUTTON_GPIO = 16
global currentSpeed
currentSpeed=0
global distance
distance=0
tickTimes=[]
prevTime=0
tickPos=0
prevTime=0
loopTime = 0
lastHallPulse = 0
countIntervals = 0
avgdT=1
distTicks = 0
in1 = 24
in2 = 23
en = 25

def signal_handler(sig, frame):
    GPIO.cleanup()
    sys.exit(0)

def button_pressed_callback(channel):
    WHEEL_CIRC = 0.12
    WHEEL_TICKS= 1
    TICK_DIST = 0.13
    global prevTime
    current=time.time()
    #tickPos %= WHEEL_TICKS
    global avgdT
    global distance
    avgdT= current - prevTime
    prevTime=current
    global lastHallPulse
    lastHallPulse = current
    distance= distance+0.12
    if(distance>=100):
        p.ChangeDutyCycle(0)   
        time.sleep(0.5)
        print("Ana el AFRIKI")
        exit()
           
def updateSpeed():   
    global currentSpeed
    currentSpeed =  0.12 /avgdT
    if(time.time() - lastHallPulse > 1):
        currentSpeed = 0
    return currentSpeed
GPIO.setmode(GPIO.BCM)
    
GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down=GPIO.PUD_UP)

GPIO.add_event_detect(BUTTON_GPIO, GPIO.FALLING, 
            callback=button_pressed_callback, bouncetime=110)
    
signal.signal(signal.SIGINT, signal_handler)

V_des = 0
flag_Sp = 0
t=10
def callback_Des_Sp(data):
    global V_des
    V_des = data.data
sub1 = rospy.Subscriber('/Long_Velocity', Float32, callback_Des_Sp)

GPIO.setmode(GPIO.BCM)
GPIO.setup(in1,GPIO.OUT)
GPIO.setup(in2,GPIO.OUT)
GPIO.setup(en,GPIO.OUT)
GPIO.output(in1,GPIO.LOW)
GPIO.output(in2,GPIO.LOW)
p=GPIO.PWM(en,100)
p.start(0) 
GPIO.output(in1,GPIO.HIGH)
GPIO.output(in2,GPIO.LOW)
cSpeed=0
time.sleep(5)
rate = rospy.Rate(100)#1600
while True:
#     rate = rospy.Rate(t)
    start=time.time()
    Kp=1.8
    values = range(10)
    for i in values:
        cSpeed=updateSpeed()+cSpeed #use ros here
    cSpeed=cSpeed/10
    cSpeed=round(cSpeed,1)
    global V_des
    e=((V_des-cSpeed)/0.9)*100
    P=Kp*e+70
    if P >100:
        P=100
    if P<0:
        P=0
    round(P)
    p.ChangeDutyCycle(100)    
    pub1.publish(round(cSpeed,2))
    pub2.publish(round(distance,2))
    end=time.time()
    t=end-start
    #print("\ntime Long"+str(t))
    rate.sleep()
 

