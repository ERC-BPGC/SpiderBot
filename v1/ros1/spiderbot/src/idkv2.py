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

l1j1 = 90
l1j2 = 100
l1j3 = 155
l2j1 = 90
l2j2 = 100
l2j3 = 155
l3j1 = 90
l3j2 = 100
l3j3 = 155

# Variables for leg angles for leg 4 5 6
l6j3 = 155
l6j2 = 100
l6j1 = 90
l5j3 = 155
l5j1 = 90
l5j2 = 100

l4j3 = 155
l4j2 = 100
l4j1 = 90

msg=Float32MultiArray()
msg2=Float32MultiArray()
j=[l1j1,l1j2,l1j3,l2j1,l2j2,l2j3,l3j1,l3j2,l3j3]
# j2 starts with l6j3
j2=[l6j3,l6j2,l6j1,l5j3,l5j2,l5j1,l4j3,l4j2,l4j1]

count = 0
increment = 10

while not rospy.is_shutdown():
    if count == 0:
	#first steps
        #j[1] += 5 #first raise leg1 by 5deg
        j[0] += increment # move leg forward

        #j2[1] += 5 #leg 4
        j2[8] += increment

        #j2[4] += 5 #leg5
        j2[5] += increment
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
    
    if count == 1:
        #j[4] += 5 #first raise leg2 by 5deg
        j[3] += increment #second move leg forward

        #j[7] += 5 #leg 3
        j[6] += increment

        #j2[7] += 5 #leg6
        j2[2] += increment
    # ---------------------------------

        j[0] -= increment # move leg1 backward

        j2[8] -= increment # moce leg4 back

        j2[5] -= increment            
        count += 1

        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()
        

    if count == 2:
        #j[1] += 5 #first raise leg1 by 5deg
        j[0] += increment #second move leg forward

        #j2[1] += 5 #leg 4
        j2[8] += increment

        #j2[4] += 5 #leg6
        j2[5] += increment
        #-----------------------------------

        j[3] -= increment #second move leg backward

        j[6] -= increment

        j2[2] -= increment

        count -= 1

        msg.data = j
        msg2.data = j2

        print(msg)
        print(msg2)
        print(count)
        pub.publish(msg)
        pub2.publish(msg2)
        r.sleep()
        
    # print(msg)
    # print(msg2)r.sleep()
    # print(count)
    # pub.publish(msg)
    # pub2.publish(msg2)
    # r.sleep()
