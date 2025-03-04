#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class CmdVelAnalyzer:
    def __init__(self):
        rospy.init_node('cmd_vel_transform', anonymous=True)
        
        # Subscriber to the /cmd_vel topic
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)

    def cmd_vel_callback(self, msg):
        # Analyze the linear and angular velocity components
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        
        # Determine movement direction based on linear.x and angular.z
        if linear_velocity > 0:
            print("Moving Forward")
        elif linear_velocity < 0:
            print("Moving Backward")
        else:
            print("No Forward/Backward movement")

        if angular_velocity > 0:
            print("Turning Right")
        elif angular_velocity < 0:
            print("Turning Left")
        else:
            print("No Left/Right Turning")

if __name__ == '__main__':
    try:
        analyzer = CmdVelAnalyzer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass