#!/usr/bin/env python3
import rospy
from cmath import pi
import numpy as np
from std_msgs.msg import Float32MultiArray

rospy.init_node("walk")

pub=rospy.Publisher("/leg123",Float32MultiArray,queue_size = 5)
pub2=rospy.Publisher("/leg456",Float32MultiArray,queue_size = 5)

rate = 0.5
r = rospy.Rate(rate)
msg=Float32MultiArray()
msg2=Float32MultiArray()

# Variables for leg angles for leg 1 2 3
l1j1 = 90
l1j2 = 85
l1j3 = 145
l2j1 = 90
l2j2 = 85
l2j3 = 145
l3j1 = 90
l3j2 = 85
l3j3 = 145

# Variables for leg angles for leg 4 5 6
l4j1=90
l4j2=85
l4j3=145
l5j1=90
l5j2=85
l5j3=145
l6j1=90
l6j2=85
l6j3=145

msg=Float32MultiArray()
msg2=Float32MultiArray()
j=[l1j1,l1j2,l1j3,l2j1,l2j2,l2j3,l3j1,l3j2,l3j3]
#   0     1    2    3    4    5   6     7    8  

# j2 starts with l6j3
j2=[l6j3,l6j2,l6j1,l5j3,l5j2,l5j1,l4j3,l4j2,l4j1]
#     0    1    2   3    4     5    6    7    8


count = 0
increment = 10

while not rospy.is_shutdown():
    if count == 0:
    # legs 1 3 5 rise

        #l1j2
        j[1] = 60

        #l3j2
        j[7] = 60

        #l5j2
        j2[4] = 60
        #-----------------------------------
        count += 1
        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        count += 1
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()

    if count == 1:
    # legs 1 3 5 move forward

        #l1j1
        j[0] = 120

        #l3j1
        j[6] = 90

        #l3j2
        j[7]=90

        #l5j3
        j2[3] = 145

        #l5j1
        j2[5]=70

    # ---------------------------------          
        count += 1
        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        count += 1
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()


    if count == 2:
    # legs 1 3 5 touch down

        #l1j2
        j[1] = 100

        #l3j2
        j[7] = 100

        #l5j2
        j2[4] = 100

        #l1j1
        j[0]=90

        #l1j3
        j[2]=145

        #l2j1
        j[3]=120

        #l2j2
        j[4]=60

        #l2j3
        j[5]=145

        #l3j1
        j[6]=90

        #l3j3
        j[8]=145

        #l4j1
        j2[8]=60
        #l4j2
        j2[7]=60
        #l4j3
        j2[6]=145

        #l5j1
        j2[5]=90

        #l5j2
        j2[4]=100

        #l5j3
        j2[3]=145

        #l6j1
        j2[2]=90

        #l6j2
        j2[1]=60
        #l6j3
        j2[0]=145

        #-----------------------------------
        count += 1
        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        count += 1
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()


    if count == 3:
        #l6j3
        j2[0]=175


        #-----------------------------------
        count += 1
        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()

    if count == 4:
    

        #l2j2
        j[4] = 85

        #l4j2
        j2[7] = 85

        #l6j2
        j2[1] = 85
        #-----------------------------------
        count += 1
        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        count = 0
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()
    # print(msg)
    # print(msg2)r.sleep()
    # print(count)
    # pub.publish(msg)
    # pub2.publish(msg2)
    # r.sleep()