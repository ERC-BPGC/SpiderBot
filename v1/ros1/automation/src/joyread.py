#!/usr/bin/env python3

import numpy as np
import rospy
from cmath import pi
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy
state=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
rospy.init_node("controller")

def callback(msg):
    global state
    if np.count_nonzero(msg.buttons)!= 0:
        if msg.buttons[3] == 1: # forward
           state=[30,-30,30,30,30,30,0,20,0,20,-60,20,60,20,0,20,0,20,1,1]
        if msg.buttons[2] == 1: # right
           state=[30,30,-30,30,-30,-30,-60,20,60,20,0,20,0,20,0,20,0,20,1,1]
        if msg.buttons[0] == 1: # left
           state=[-30,-30,30,-30,30,30,0,20,0,20,0,20,-60,20,0,20,0,20,1,1]
        if msg.buttons[1] == 1: # back
           state=[-30,30,-30,-30,-30,-30,60,20,0,20,0,20,0,20,0,20,0,20,1,1]
        if msg.buttons[4] == 1: # rotate
           state=[30,-30,30,-30,-30,-30,0,20,0,20,0,20,0,20,0,20,0,20,1,1]
        if msg.buttons[5] == 1:   #rotate
           state=[-30,30,-30,30,30,30,0,20,0,20,0,20,0,20,0,20,0,20,1,1]
    else:
           state=[0,0,0,0,0,0,90,90,90,90,90,90,90,90,90,90,90,90,0,0]




sub = rospy.Subscriber("/joy",Joy,callback)
pub = rospy.Publisher("/controller",Float32MultiArray,queue_size = 5)


r = rospy.Rate(5)

while True:
        msg = Float32MultiArray()
        msg.data =state[0:]
        pub.publish(msg)
        if rospy.is_shutdown():
            break

        #rospy.sleep(5)
        r.sleep()
        print('Repeating')
