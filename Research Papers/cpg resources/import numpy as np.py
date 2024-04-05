#!/usr/bin/env python3

import numpy as np
import rospy
from cmath import pi
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
rospy.init_node("controller")

def callback(msg):
    global file, lines
    
    if msg.buttons[0] == 1: # forward
        state[0]=1
    if msg.buttons[1] == 1: # right
        state[1]=1
    if msg.buttons[2] == 1: # left
        state[2]=1
    if msg.buttons[3] == 1: # back
         state[3]=1
    if msg.buttons[9] == 1: # start    
         state[4]=1

sub = rospy.Subscriber("/joy",Joy,callback)
pub = rospy.Publisher("/controller",Float32MultiArray,queue_size = 5)


r = rospy.Rate(5)

while True:
 
        msg = Float32MultiArray()
        pub.publish(msg)
        
        if rospy.is_shutdown():
            break
    
	#rospy.sleep(5)
        r.sleep()
        print('Repeating')

                                                                  

