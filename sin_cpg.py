#!/usr/bin/env python3

import math
import numpy as np
import matplotlib.pyplot as plt
import rospy
from std_msgs.msg import Float64

pi = 3.141569
amp = 3
camp = 0.8
phase = [pi, -pi]
offset = np.array([0,0])
coff = 0.1
frequency = 0.1
dt = 0.0005
result = 0
time = np.arange(0,100,dt)
iterations = 1000000

rospy.init_node("CPG")
rate = rospy.Rate(76800)

pub1 = rospy.Publisher("leg1",Float64,queue_size=10)
pub2 = rospy.Publisher("leg2",Float64,queue_size=10)

def dp(frequency, idx, phase, phase_offset_target):
    if idx == 0:
        return 2 * pi * frequency + 1 * amp * math.sin(phase[1]-phase[0]-phase_offset_target)
    else:
        return 2 * pi * frequency + 1 * amp * math.sin(phase[0]-phase[1]+phase_offset_target)

def da(amp_target):
    return camp * (amp_target - amp)

def do(offset_target):
    return coff * (offset_target - offset)

def update_values(amp_target = 1, offset_target = np.array([0,0]), frequency_target = 0.1, phase_offset_target = pi):
    global phase, amp, offset
    for i in range(2):
        phase[i] = phase[i] + dp(frequency_target, i, phase, phase_offset_target) * dt
        
    amp = amp + da(amp_target) * dt
    offset = offset + do(offset_target)
    
def output():
    return amp * np.sin(phase) + offset

outputs = []
n1 = []
n2 = []


# for i in range(iterations):
while not rospy.is_shutdown():
    # if i < iterations / 3:
    #     update_values()
    # elif i < 2 * iterations / 3:
    #     update_values(2,offset_target = np.array([1,5]),frequency_target = 0.2,phase_offset_target = pi + 1.57)
    # else:
    #     update_values(offset_target = np.array([1,1]), phase_offset_target = 2*pi+1.57)
    update_values()


    result = output()
    msg1 = Float64()
    msg2 = Float64()

    msg1.data = result[0]
    msg2.data = result[1]

    pub1.publish(msg1)
    pub2.publish(msg2)
    # n1.append(result[0])
    # n2.append(result[1])
    rate.sleep()


# plt.plot(time, n1)
# plt.plot(time, n2)
# plt.show()