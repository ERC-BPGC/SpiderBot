#!/usr/bin/env python3
 
import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
 
class AStarPlanner:
    def __init__(self):
        rospy.init_node('astar_path_planner', anonymous=True)
        
        # Configuration parameters
        self.resolution = 0.1  # meters per grid cell
        self.obstacle_threshold = 0.5  # meters
        self.safety_margin = 0.3  # meters
        self.goal_tolerance = 0.1  # meters
        self.replan_interval = 1.0  # seconds
        
        # ROS interfaces
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.path_pub = rospy.Publisher('/path', Path, queue_size=1)
 
        # System state
        self.map_info = None
        self.current_pose = (0.0, 0.0, 0.0)  # x, y, yaw
        self.goal = (10.0, 10.0)  # Example goal position
        self.path = []
        self.last_replan = rospy.Time.now()
        self.initialized = False
 
    # Core planning functions
    def world_to_grid(self, world_pos):
        """Convert world coordinates to grid indices"""
        if not self.map_info:
            return (0, 0)
            
        grid_x = int((world_pos[0] - self.map_info['origin_x']) / self.resolution)
        grid_y = int((world_pos[1] - self.map_info['origin_y']) / self.resolution)
        return (
            np.clip(grid_x, 0, self.map_info['width']-1),
            np.clip(grid_y, 0, self.map_info['height']-1)
        )
 
    def grid_to_world(self, grid_pos):
        """Convert grid indices to world coordinates"""
        return (
            grid_pos[0] * self.resolution + self.map_info['origin_x'],
            grid_pos[1] * self.resolution + self.map_info['origin_y']
        )
 
    def heuristic(self, a, b):
        """Manhattan distance heuristic"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
 
    def get_neighbors(self, node):
        """Generate valid neighboring nodes"""
        neighbors = []
        for dx, dy in [(-1,0), (1,0), (0,-1), (0,1)]:
            x = node[0] + dx
            y = node[1] + dy
            if 0 <= x < self.map_info['width'] and 0 <= y < self.map_info['height']:
                neighbors.append((x, y))
        return neighbors
 
    def a_star(self, start, goal):
        """A* pathfinding algorithm implementation"""
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
                if self.map_info['data'][neighbor[1], neighbor[0]] > 50:
                    continue
                
                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
 
        rospy.logwarn("No path found to goal")
        return None
 
    def reconstruct_path(self, came_from, current):
        """Rebuild path from A* results"""
        path = [self.grid_to_world(current)]
        while current in came_from:
            current = came_from[current]
            path.append(self.grid_to_world(current))
        return list(reversed(path))
 
    # ROS callbacks
    def map_callback(self, msg):
        """Process map updates"""
        self.map_info = {
            'data': np.array(msg.data).reshape((msg.info.height, msg.info.width)),
            'origin_x': msg.info.origin.position.x,
            'origin_y': msg.info.origin.position.y,
            'width': msg.info.width,
            'height': msg.info.height
        }
        self.initialized = True
        rospy.loginfo("Map initialized")
 
    def odom_callback(self, msg):
        """Update robot pose"""
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        self.current_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            euler_from_quaternion(quaternion)[2]
        )
 
    def lidar_callback(self, msg):
        """Handle dynamic obstacle updates"""
        if not self.initialized:
            return
 
        # Update obstacle map
        for i, dist in enumerate(msg.ranges):
            if dist < self.obstacle_threshold:
                angle = msg.angle_min + i*msg.angle_increment
                x = self.current_pose[0] + dist * np.cos(angle + self.current_pose[2])
                y = self.current_pose[1] + dist * np.sin(angle + self.current_pose[2])
                self.mark_obstacle(x, y)
 
        # Replan if needed
        if (rospy.Time.now() - self.last_replan).to_sec() > self.replan_interval:
            self.plan_path()
 
    def mark_obstacle(self, x, y):
        """Mark obstacle area in costmap"""
        gx, gy = self.world_to_grid((x, y))
        margin = int(self.safety_margin / self.resolution)
        
        for dx in range(-margin, margin+1):
            for dy in range(-margin, margin+1):
                px = gx + dx
                py = gy + dy
                if 0 <= px < self.map_info['width'] and 0 <= py < self.map_info['height']:
                    self.map_info['data'][py, px] = 100
 
    # Planning and execution
    def plan_path(self):
        """Generate new path using A*"""
        if not self.initialized:
            return
 
        start = self.world_to_grid((self.current_pose[0], self.current_pose[1]))
        goal = self.world_to_grid(self.goal)
        
        self.path = self.a_star(start, goal)
        self.last_replan = rospy.Time.now()
        self.publish_path()
 
    def publish_path(self):
        """Publish current path to ROS"""
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in self.path:
            pose = PoseStamped()
            pose.pose.position = Point(*point, 0)
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
 
    def execute_path(self):
        """Main control loop for path following"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.initialized:
                rate.sleep()
                continue
                
            if not self.path:
                self.plan_path()
                rate.sleep()
                continue
 
            # Get current target point
            target = self.path[0]
            dx = target[0] - self.current_pose[0]
            dy = target[1] - self.current_pose[1]
            distance = np.hypot(dx, dy)
            
            # Create velocity command
            cmd = Twist()
            if distance < self.goal_tolerance:
                self.path.pop(0)
                if not self.path:
                    rospy.loginfo("Goal reached!")
                    self.cmd_pub.publish(Twist())
                    return
                continue
 
            # Calculate steering
            target_yaw = np.arctan2(dy, dx)
            yaw_error = target_yaw - self.current_pose[2]
            
            if abs(yaw_error) > 0.1:
                cmd.angular.z = 0.3 if yaw_error > 0 else -0.3
            else:
                cmd.linear.x = 0.2 * min(distance, 1.0)
            
            self.cmd_pub.publish(cmd)
            rate.sleep()
 
if __name__ == '__main__':
    try:
        planner = AStarPlanner()
        rospy.sleep(1)  # Wait for initial data
        planner.execute_path()
    except rospy.ROSInterruptException:
        pass