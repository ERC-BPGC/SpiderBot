#!/usr/bin/env python3
    
import rospy
import numpy as np
import heapq
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
    
class RobotPathPlanner:
    def __init__(self):
        rospy.init_node('robot_path_planner', anonymous=True)
        
        # Parameters
        self.resolution = 0.1  # Map resolution in meters/cell
        self.obstacle_threshold = 0.5  # Obstacle distance threshold
        self.safety_margin = 0.3  # Safety margin around obstacles
        self.replan_threshold = 0.5  # Replanning threshold
        self.max_linear_vel = 0.4  # Maximum linear velocity
        self.max_angular_vel = 0.6  # Maximum angular velocity
        self.acc_lim_x = 1.0  # Linear acceleration limit
        self.sim_time = 1.5  # Simulation time for trajectory scoring
        self.occdist_scale = 0.1  # Obstacle avoidance weight
        self.pdist_scale = 0.8  # Path following weight
        self.gdist_scale = 1.2  # Goal attraction weight
        
        # Subscribers and Publishers
        self.lidar_sub = rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/astar_path', Path, queue_size=10)
    
        # State variables
        self.map_data = None
        self.current_pose = (0, 0, 0)  # (x, y, yaw)
        self.goal_position = (0.5, 1)  # Example goal (y,x)
        self.path = None
        self.initialized = False
    
    def odom_callback(self, msg):
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
    
    def map_callback(self, msg):
        self.map_data = {
            'data': np.array(msg.data).reshape((msg.info.height, msg.info.width)),
            'origin': (msg.info.origin.position.x, msg.info.origin.position.y),
            'resolution': msg.info.resolution,
            'width': msg.info.width,
            'height': msg.info.height
        }
        self.initialized = True
        rospy.loginfo("Map received: %dx%d grid" % (msg.info.width, msg.info.height))
    
    def lidar_callback(self, msg):
        if self.map_data is None:
            return
    
        ranges = np.array(msg.ranges)
        angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
        
        for i, distance in enumerate(ranges):
            if distance < self.obstacle_threshold:
                angle = angles[i] + self.current_pose[2]
                x = self.current_pose[0] + distance * np.cos(angle)
                y = self.current_pose[1] + distance * np.sin(angle)
                self.mark_obstacle(x, y)
    
        if self.path and any(self.is_obstructed(point) for point in self.path):
            rospy.loginfo("Obstacle detected - replanning path")
            self.plan_path()
    
    def mark_obstacle(self, x, y):
        grid_x, grid_y = self.world_to_grid((x, y))
        margin = int(self.safety_margin / self.map_data['resolution'])
        
        for dx in range(-margin, margin+1):
            for dy in range(-margin, margin+1):
                dist = np.hypot(dx, dy)
                cost = int(100 * np.exp(-dist**2/(2*(margin/2)**2)))
                px = grid_x + dx
                py = grid_y + dy
                if 0 <= px < self.map_data['width'] and 0 <= py < self.map_data['height']:
                    self.map_data['data'][py, px] = max(
                        self.map_data['data'][py, px], 
                        min(cost, 100)
                    )
    
    def plan_path(self):
        start = (self.current_pose[0], self.current_pose[1])
        self.path = self.a_star(start, self.goal_position)
        self.publish_path()
    
    def a_star(self, start, goal):
        start_grid = self.world_to_grid(start)
        goal_grid = self.world_to_grid(goal)
        
        open_set = []
        heapq.heappush(open_set, (0, start_grid))
        
        came_from = {}
        g_score = {start_grid: 0}
        f_score = {start_grid: self.heuristic(start_grid, goal_grid)}
    
        while open_set:
            current = heapq.heappop(open_set)[1]
    
            if current == goal_grid:
                return self.reconstruct_path(came_from, current)
    
            for neighbor in self.get_neighbors(current):
                if self.map_data['data'][neighbor[1], neighbor[0]] > 50:
                    continue
                
                tentative_g = g_score[current] + 1  # Uniform cost for all movements
    
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic(neighbor, goal_grid)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
    
        return None
    
    def world_to_grid(self, world_pos):
        grid_x = int((world_pos[0] - self.map_data['origin'][0]) / self.map_data['resolution'])
        grid_y = int((world_pos[1] - self.map_data['origin'][1]) / self.map_data['resolution'])
        return (
            np.clip(grid_x, 0, self.map_data['width']-1),
            np.clip(grid_y, 0, self.map_data['height']-1)
        )
    
    def grid_to_world(self, grid_pos):
        return (
            grid_pos[0] * self.map_data['resolution'] + self.map_data['origin'][0],
            grid_pos[1] * self.map_data['resolution'] + self.map_data['origin'][1]
        )
    
    def heuristic(self, a, b):
        return np.hypot(b[0]-a[0], b[1]-a[1])
    
    def get_neighbors(self, pos):
        return [(pos[0]+dx, pos[1]+dy) 
                for dx,dy in [(-1,0),(1,0),(0,-1),(0,1),
                                (-1,-1),(-1,1),(1,-1),(1,1)]]
    
    def reconstruct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(self.grid_to_world(current))
            current = came_from[current]
        return path[::-1]
    
    def is_obstructed(self, point):
        grid_x, grid_y = self.world_to_grid(point)
        return self.map_data['data'][grid_y, grid_x] > 50
    
    def publish_path(self):
        if not self.path:
            return
            
        path_msg = Path()
        path_msg.header.stamp = rospy.Time.now()
        path_msg.header.frame_id = "map"
        
        for point in self.path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            path_msg.poses.append(pose)
            
        self.path_pub.publish(path_msg)
        rospy.loginfo("Published path with %d waypoints" % len(path_msg.poses))
    
    def move_to_goal(self):
        if not self.initialized:
            rospy.logwarn("Waiting for initialization...")
            return
    
        if not self.path:
            self.plan_path()
            if not self.path:
                rospy.logwarn("No path found - initiating rotation recovery")
                self.perform_rotation_scan()
                return
    
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            if not self.path:
                rospy.loginfo("Path completed")
                self.cmd_vel_pub.publish(Twist())  # Stop the robot
                return
    
            # Select lookahead point
            lookahead_idx = min(5, len(self.path)-1)
            target = self.path[lookahead_idx]
    
            # Vector field guidance
            target_vector = np.array(target) - np.array(self.current_pose[:2])
            robot_vector = np.array([np.cos(self.current_pose[2]), np.sin(self.current_pose[2])])
            
            cross_product = np.cross(robot_vector, target_vector)
            dot_product = np.dot(robot_vector, target_vector)
            heading_error = np.arctan2(cross_product, dot_product)
    
            distance = np.linalg.norm(target_vector)
            
            twist = Twist()
            if abs(heading_error) > 0.1:
                twist.angular.z = np.clip(heading_error, -self.max_angular_vel, self.max_angular_vel)
            else:
                twist.linear.x = self.max_linear_vel * min(distance, 1.0)
            
            self.cmd_vel_pub.publish(twist)
            rospy.loginfo(f"CMD_VEL - Linear: {twist.linear.x:.2f}, Angular: {twist.angular.z:.2f}")
    
            # Check if we've reached the current waypoint
            if distance < 0.1:
                self.path.pop(0)
                if not self.path:
                    rospy.loginfo("Goal reached!")
                    self.cmd_vel_pub.publish(Twist())  # Stop the robot
                    return
            
            rate.sleep()
    
    def perform_rotation_scan(self):
        twist = Twist()
        twist.angular.z = 0.5  # Rotate slowly
        
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        
        while (rospy.Time.now() - start_time).to_sec() < 4.0:  # Rotate for 4 seconds
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        self.cmd_vel_pub.publish(Twist())  # Stop rotation
        self.plan_path()  # Try to plan a path again
    
if __name__ == '__main__':
    try:
        planner = RobotPathPlanner()
        rospy.sleep(1)  # Wait for initialization
        while not rospy.is_shutdown():
            planner.move_to_goal()
            rospy.sleep(1)
    except rospy.ROSInterruptException:
        pass