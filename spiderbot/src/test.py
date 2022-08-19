#!/usr/bin/env python3
import rospy
from cmath import pi
import numpy as np
from std_msgs.msg import Float32MultiArray

rospy.init_node("test")

def convangs(arr):
	jts = np.array(np.float_(arr))
	
	return jts #returns a np array after converting string array to a float array

def normalize_pi(j):
	for j_ in j:
		while 1:
			if j_>360:
				j_-=360
			elif j_<0:
				j_+=360
			else:
				break

# file = open("/home/erc/catkin_ws/src/spiderbot/src/oneleg.txt",'r')
# lines = file.readlines()

pub=rospy.Publisher("/leg123",Float32MultiArray,queue_size = 5)
pub2=rospy.Publisher("/leg456",Float32MultiArray,queue_size = 5)

r = rospy.Rate(10)
#for line in lines:
	#x = line.split(",",)
	#x.pop(-1)
	
	# j = convangs(x) #converts string array to float array
	# j = j*(180/pi)  #multiply each element of np array with (180/pi)
	# # j = normalize_pi(j)

msg=Float32MultiArray()
msg2=Float32MultiArray()
j=[90,85,145,90,85,145,90,85,145]
j2=[90,85,145,90,85,145,90,85,145]
#j=[90,85,145,90,50,0,90,85,145]
#j2=[90,50,0,90,85,145,90,50,0]
#j3=[70,85,145,70,85,145,70,85,145,70,85,145,70,85,145,70,85,145]
#j5=[60,75,150,60,75,150,60,75,150]
#j6=[60,75,165,60,75,165,60,75,165]

count = 0

while not rospy.is_shutdown():
	if count == 0:	
		msg.data = j
		msg2.data = j2
		#count=1
	elif count == 1:
		msg.data = j3
		count = 2
	elif count == 2:
		msg.data = j
		count=3
	else:
		msg.data = j3
		count=0
		
	print(msg)
	print(count)
	pub.publish(msg)
	pub2.publish(msg2)
	rospy.sleep(7)


# file.close()

																	
