#!/usr/bin/env python3

import numpy as np
import rospy
from cmath import pi
from std_msgs.msg import Float32MultiArray

rospy.init_node("test")

def convangs(arr):
    jts = np.array(np.float_(arr))
    return jts #returns a np array after converting string array to a float array

file = open("4-2_walk_forward.txt",'r')
lines = file.readlines()


pub = rospy.Publisher("/leg123",Float32MultiArray,queue_size = 5)
pub2=rospy.Publisher("/leg456",Float32MultiArray,queue_size=5)

r = rospy.Rate(5)

while True:
    for index, line in enumerate(lines):
        # print("Line {}: {}".format(index, line.strip()))
        #break
        # print(line[index])
        """ Add servo code here """
        
        x = line.split(",")
        #x.pop(-1)
        #b = float(x)

        for i in range(18):
            b = float(x[i])
            x[i] = b
	    
	# j = x[:9]
	# j2 = x[9:]

	# x = [90,85,145,90,85,145,90,85,145,90,85,145,90,85,145,90,85,145,]  
        msg = Float32MultiArray()
        msg2 = Float32MultiArray()
        
        msg.data = x[:9]
        msg2.data =x[9:]
        
        pub.publish(msg)
        pub2.publish(msg2)
        
        if rospy.is_shutdown():
            break
    
	#rospy.sleep(5)
        r.sleep()
    print('Repeating')

file.close()                                                                    
