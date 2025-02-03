#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import numpy as np
import heapq
from tf.transformations import euler_from_quaternion

class RobotPathPlanner:
    def __init__(self):
        rospy.init_node('robot_path_planner', anonymous=True)
        
        # Parameters
        self.grid_size = 20  # Size of the grid (can be changed based on map resolution)
        self.obstacle_threshold = 0.5  # Threshold for obstacle detection from LiDAR in meters
        
        # Subscribers and Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize variables
        self.map_data = None
        self.current_position = (0, 0)
        self.goal_position = (self.grid_size - 1, self.grid_size - 1)
        self.current_orientation = 0.0
        self.obstacles = []
        
    def odom_callback(self, data):
        # Update the robot's position and orientation from Odometry data
        position = data.pose.pose.position
        orientation = data.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        self.current_position = (position.x, position.y)
        self.current_orientation = yaw

    def lidar_callback(self, data):
        # Process LiDAR data to detect obstacles
        ranges = np.array(data.ranges)
        angles = np.linspace(data.angle_min, data.angle_max, len(ranges))
        
        self.obstacles = []
        for i in range(len(ranges)):
            if 0 < ranges[i] < self.obstacle_threshold:  # Ignore invalid or too far readings
                x = self.current_position[0] + ranges[i] * np.cos(angles[i] + self.current_orientation)
                y = self.current_position[1] + ranges[i] * np.sin(angles[i] + self.current_orientation)
                self.obstacles.append((x, y))

    def map_callback(self, data):
        # Store map data
        self.map_data = np.array(data.data).reshape((data.info.height, data.info.width))
        self.map_resolution = data.info.resolution
        self.map_origin = (data.info.origin.position.x, data.info.origin.position.y)

    def a_star(self, start, goal):
        # A* algorithm implementation for pathfinding
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic(start, goal)}

        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.get_neighbors(current):
                if not self.is_valid(neighbor):
                    continue
                
                tentative_g_score = g_score[current] + 1

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + self.heuristic(neighbor, goal)
                    
                    if neighbor not in [i[1] for i in open_set]:
                        heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, position):
        x, y = position
        neighbors = [(x + dx, y + dy) for dx in [-1, 0, 1] for dy in [-1, 0, 1]]
        return [n for n in neighbors if n != position]

    def is_valid(self, position):
        x, y = position
        map_x = int((x - self.map_origin[0]) / self.map_resolution)
        map_y = int((y - self.map_origin[1]) / self.map_resolution)

        return (0 <= map_x < self.map_data.shape[1] and 
                0 <= map_y < self.map_data.shape[0] and 
                self.map_data[map_y][map_x] == 0)

    def reconstruct_path(self, came_from, current):
        total_path = [current]
        
        while current in came_from:
            current = came_from[current]
            total_path.append(current)
        
        return total_path[::-1]

    def move_robot(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.map_data is not None:
                # Convert goal to map coordinates
                goal_x = self.goal_position[0] * self.map_resolution + self.map_origin[0]
                goal_y = self.goal_position[1] * self.map_resolution + self.map_origin[1]

                path = self.a_star(self.current_position, (goal_x, goal_y))
                
                for step in path:
                    twist = Twist()
                    
                    # Move to the next step using proportional control
                    dx = step[0] - self.current_position[0]
                    dy = step[1] - self.current_position[1]
                    distance = np.sqrt(dx**2 + dy**2)
                    angle_to_goal = np.arctan2(dy, dx)

                    twist.linear.x = min(0.2, distance)  # Forward velocity
                    twist.angular.z = 2.0 * (angle_to_goal - self.current_orientation)
                    self.cmd_vel_pub.publish(twist)
                    rospy.sleep(1)  # Sleep for a second to simulate movement

            rate.sleep()

if __name__ == '__main__':
    try:
        planner = RobotPathPlanner()
        planner.move_robot()
    except rospy.ROSInterruptException:
        pass
