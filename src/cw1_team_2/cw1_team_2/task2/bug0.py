import rclpy
import math
import numpy as np
from rclpy.node import Node
from cw1_team_2.task1.cw1_edge_follower import AdvancedEdgeFollowerNodes
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from cw1_team_2.utils.utils import is_intersected, get_rect_marker

"""
MIT License

Copyright (c) 2025 Comp0244-Team2

Authors: 
- Hongbo Li (Code)
- Yiyang Jia (Code)
- Xinyun Mo (Testing)

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""

class BugPlanner(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info(f"{name} Node Started")
        self.edge_follower = AdvancedEdgeFollowerNodes()
        
        # Declare Parameters
        self.declare_parameter("goal_x", 0.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_theta", 0.0)

        # Constants
        self.UPDATE_RATE = 0.2  # [s]
        self.WAYPOINT_DISTANCE = 0.3  # [m]
        self.WAYPOINT_TOLERANCE = 0.1 # [m]
        self.OBSTACLE_DISTANCE = 1.0  # [m]

        # Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_edges = []
        self.NEW_GOAL = True
        self.last_goal = [self.current_x, self.current_y, self.current_theta]

        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 1)
        self.rect_pub = self.create_publisher(Marker, 'rectangle', 10)

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.edge_follower.odom_callback,
            10)
            
        self.line_sub = self.create_subscription(
            Marker,
            'local_map_lines',
            self.edge_follower.line_callback,
            10)
            
        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)
        
        self.get_logger().info('Edge Follower node initialized')
        self.get_logger().info('Bug0 node initialized')

    def is_goal_reached(self):
        """Check if the goal is reached"""
        distance = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
        return distance < self.WAYPOINT_TOLERANCE
    
    def is_edge_crossed(self, edges, edges1):
        '''Check if any edge in two sets of edges are crossed'''
        for i in range(len(edges)):
            for j in range(len(edges1)):
                
                if is_intersected(edges[i][0], edges[i][1], edges1[j][0], edges1[j][1]):
                    return True
        return False
    
    def is_obstacle_detected(self, theta_livox, current_edges):
        """
        Check if there is an obstacle in the way towards the goal
        Args:
        theta_livox: The angle between the robot and the goal in the base_link frame
        current_edges: The edges detected by the LiDAR in the base_link frame
        """
        # Check if any edge in the way (within a rectangular area of distance OBSTACLE_DISTANCE towards the goal and width of the robot)
        # If there is an edge in the way, follow the edge
        rectangle_width = 0.3
        rect = np.zeros((4,2))
        rect[0] = [1/2*rectangle_width*math.cos(theta_livox+np.pi/2), 1/2*rectangle_width*math.sin(theta_livox+np.pi/2)]
        rect[1] = [rect[0][0] + self.OBSTACLE_DISTANCE*math.cos(theta_livox), rect[0][1] + self.OBSTACLE_DISTANCE*math.sin(theta_livox)]
        rect[2] = [rect[1][0] + rectangle_width*math.cos(theta_livox - np.pi/2), rect[1][1] + rectangle_width*math.sin(theta_livox - np.pi/2)]
        rect[3] = [rect[2][0] + self.OBSTACLE_DISTANCE*math.cos(theta_livox + np.pi), rect[2][1] + self.OBSTACLE_DISTANCE*math.sin(theta_livox + np.pi)] 

        # Publish the rectangle
        rect_msg = get_rect_marker(rect, self.get_clock().now().to_msg())
        self.rect_pub.publish(rect_msg)
        
        # Check if the edge crosses the rectangle
        rect_edges = []
        for i in range(len(rect)):
            rect_edges.append(np.array([rect[i], rect[(i+1)%4]]))
        if self.is_edge_crossed(current_edges, rect_edges):
            # self.get_logger().info("Edge Detected")
            return True
        else :
            # self.get_logger().info("No Edge Detected")
            return False
        
    def move_to_goal(self):
        """
        Move the robot towards the goal in two different ways, depending on the presence of obstacles
        1. If there is no obstacle in the way, move towards the goal
        2. If there is an obstacle in the way, follow the edge of the obstacle until the robot can move towards the goal
        """
        # Get Direction to the goal
        goal_livox = self.edge_follower.transform_to_base_link([self.goal_x, self.goal_y]) 
        theta_livox = math.atan2(goal_livox[1], goal_livox[0])
        Count = 0
        # Check if there is an obstacle in the way
        while True:
            if not self.is_obstacle_detected(theta_livox, self.current_edges):
                Count += 1
                if Count == 100 or not self.timer.is_canceled(): # No obstacle detected for 100 iterations or robot already moving
                    break
            else:
                Count = 0
            # pause the timer
            if not self.timer.is_canceled():
                self.timer.cancel()
                self.edge_follower.last_closest_point = None
                self.edge_follower.closest_edge_point = None
                self.edge_follower.current_waypoint = None

            rclpy.spin_once(self.edge_follower)
            self.update_data()
            goal_livox = self.edge_follower.transform_to_base_link([self.goal_x, self.goal_y])
            theta_livox = math.atan2(goal_livox[1], goal_livox[0])

            
        if self.timer.is_canceled():
            self.get_logger().info("Obstacle Cleared") 
            self.timer.reset()

        # Move towards the goal with self.WAYPOINT_DISTANCE
        next_waypoint = [self.WAYPOINT_DISTANCE*math.cos(theta_livox), self.WAYPOINT_DISTANCE*math.sin(theta_livox),theta_livox]
        self.edge_follower.publish_visualizations(next_waypoint)
        next_waypoint = self.edge_follower.transform_to_camera_init(next_waypoint)
        waypoint_msg = Pose2D()
        waypoint_msg.x = next_waypoint[0]
        waypoint_msg.y = next_waypoint[1]
        waypoint_msg.theta = next_waypoint[2]
        self.waypoint_pub.publish(waypoint_msg)
        return

    def update_data(self):
        """Update the current position of the robot"""
        # Get the current position of the robot
        self.current_x = self.edge_follower.current_x
        self.current_y = self.edge_follower.current_y
        self.current_theta = self.edge_follower.current_orientation

        # Get Parameters (Goal)
        self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
        self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
        self.goal_theta = self.get_parameter("goal_theta").get_parameter_value().double_value
        self.goal = [self.goal_x, self.goal_y, self.goal_theta]
        if self.goal != self.last_goal:
            self.get_logger().info(f"New Goal: {self.goal_x}, {self.goal_y}, {self.goal_theta}")
            self.NEW_GOAL = True
        self.last_goal = [self.goal_x, self.goal_y, self.goal_theta]

        # Get Current Edges
        self.current_edges = self.edge_follower.current_edges

        return
    
    def timer_callback(self):
        """Main loop of the bug0 planner"""
        if not self.edge_follower.is_odom_received:
            return
        
        # Update Data
        self.update_data()

        # Check if the goal is reached
        if self.is_goal_reached():
            self.get_logger().info("Goal Reached")
            self.NEW_GOAL = False

            return
        if self.NEW_GOAL:
            self.move_to_goal()

    
def main(args=None):
    rclpy.init(args=args)
    node = BugPlanner("Bug0")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
