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

# Variables for leg angles for leg 1 2 3
l1j1 = 90
l1j2 = 110
l1j3 = 105
l2j1 = 90
l2j2 = 110
l2j3 = 105
l3j1 = 90
l3j2 = 110
l3j3 = 105

# Variables for leg angles for leg 4 5 6
l6j3 = 105
l6j2 = 110
l6j1 = 90
l5j3 = 105
l5j1 = 90
l5j2 = 110

l4j3 = 105
l4j2 = 110
l4j1 = 90

# Variables for leg angles for leg 1 2 3
#l1j1 = 0
#l1j2 = 0
#l1j3 = 0
#l2j1 = 0
#l2j2 = 0
#l2j3 = 0
#l3j1 = 0
#l3j2 = 0
#l3j3 = 0

# Variables for leg angles for leg 4 5 6
#l6j3 = 0
#l6j2 = 0
#l6j1 = 0
#l5j3 = 0
#l5j1 = 0
#l5j2 = 0

#l4j3 = 0
#l4j2 = 0
#l4j1 = 0

leg1step1 = [90, 60, 145, 120, 85, 145, 90, 60, 145]
leg2step1 = [100, 85, 90, 145, 60, 90, 145, 85, 60]

leg1step2 = [90, 85, 145, 90, 85, 145, 90, 85, 145]
leg2step2 = [145, 85, 90, 145, 85, 90, 145, 85, 90]

leg1step3 = [120, 85, 145, 90, 60, 145, 90, 85, 100]
leg2step3 = [145, 60, 90, 145, 85, 60, 145, 60, 90]

leg1step4 = [90, 85, 145, 90, 60, 145, 90, 85, 145]
leg2step4 = [145, 60, 90, 145, 85, 90, 145, 60, 90]


msg=Float32MultiArray()
msg2=Float32MultiArray()
j=[l1j1,l1j2,l1j3,l2j1,l2j2,l2j3,l3j1,l3j2,l3j3]
# j2 starts with l6j3
j2=[l6j3,l6j2,l6j1,l5j3,l5j2,l5j1,l4j3,l4j2,l4j1]
# j2=[90,85,145,90,85,145,90,85,145][::-1]
#j=[90,90,90,90,90,90,90,90,90]
#j2=[90,90,90,90,90,90,90,90,90]
# j=[0,0,0,0,0,0,0,0,0]
# j2=[0,0,0,0,0,0,0,0,0]
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
		msg.data = leg1step1
		msg2.data = leg2step1
		count = 2
	elif count == 2:
		msg.data = leg1step2
		msg2.data = leg2step2
		count=3
	elif count == 3:
		msg.data = leg1step3
		msg2.data = leg2step3
		count=4
	elif count == 4:
		msg.data = leg1step4
		msg2.data = leg2step4
		count = 0
		
	print(msg)
	print(msg2)
	print(count)
	pub.publish(msg)
	pub2.publish(msg2)
	rospy.sleep(2)


# file.close()

																	
