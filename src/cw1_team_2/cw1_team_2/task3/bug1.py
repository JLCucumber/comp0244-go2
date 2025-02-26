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
        self.WAYPOINT_DISTANCE = 0.2  # [m]
        self.WAYPOINT_TOLERANCE = 0.5 # [m]
        self.OBSTACLE_DISTANCE = 0.75  # [m]
        self.STARTPOINT_TOLERANCE = 1.0 # [m]

        # Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_edges = []
        self.NEW_GOAL = False
        self.distance_to_goal = 0.0
        self.loop_start_point = [0.0, 0.0]
        self.leaving_point = [0.0, 0.0]
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
        self.get_logger().info('Bug1 node initialized')

    def is_goal_reached(self):
        """Check if the goal is reached"""
        distance = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
        return distance < self.WAYPOINT_TOLERANCE
    
    def is_edge_crossed(self, edges, edges1):
        for i in range(len(edges)):
            for j in range(len(edges1)):
                
                if is_intersected(edges[i][0], edges[i][1], edges1[j][0], edges1[j][1]):
                    return True
        return False
    
    # def is_in_area(self, point, edges):
    #     for i in range(len(edges)):
    
    def is_obstacle_detected(self, theta_livox, current_edges):
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
        # Get Direction to the goal
        goal_livox = self.edge_follower.transform_to_base_link([self.goal_x, self.goal_y]) 
        theta_livox = math.atan2(goal_livox[1], goal_livox[0])
        LOOP_FINISHED = True 
        LOOP_STARTED = False
        # Check if there is an obstacle in the way
        while True:
            # Check if there is an obstacle in the way while moving towards the goal
            if self.is_obstacle_detected(theta_livox, self.current_edges) and not self.timer.is_canceled(): # Obstacle detected and robot moving towards the goal
                LOOP_FINISHED = False
                # pause the timer and start following the edge
                self.timer.cancel()
                self.loop_start_point = [self.current_x, self.current_y] # Record the start point of the loop
                min_distance = np.inf

                # Reset the edge follower
                self.edge_follower.last_closest_point = None
                self.edge_follower.closest_edge_point = None
                self.edge_follower.current_waypoint = None
            
            elif not self.timer.is_canceled(): # No obstacle detected and robot moving towards the goal
                break
            
            # Check if the loop is finished and the robot is close to the leaving point
            if LOOP_FINISHED and math.sqrt((self.current_x - self.leaving_point[0])**2 + (self.current_y - self.leaving_point[1])**2) < self.WAYPOINT_TOLERANCE: 
                    break
            
            # Apply the edge following algorithm
            rclpy.spin_once(self.edge_follower)
            
            # Update Data
            self.update_data()
            goal_livox = self.edge_follower.transform_to_base_link([self.goal_x, self.goal_y])
            theta_livox = math.atan2(goal_livox[1], goal_livox[0])
            if self.distance_to_goal < min_distance:
                min_distance = self.distance_to_goal
                self.leaving_point = [self.current_x, self.current_y]
            
            # Check if the robot has started the loop
            if not LOOP_STARTED and math.sqrt((self.current_x - self.loop_start_point[0])**2 + (self.current_y - self.loop_start_point[1])**2) > self.STARTPOINT_TOLERANCE:
                LOOP_STARTED = True

            # Check if the robot has finished the loop
            if LOOP_STARTED and math.sqrt((self.current_x - self.loop_start_point[0])**2 + (self.current_y - self.loop_start_point[1])**2) < self.STARTPOINT_TOLERANCE:
                LOOP_FINISHED = True
            

        
        # Resume robot moving towards the goal
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
        self.distance_to_goal = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
        if self.goal != self.last_goal:
            self.get_logger().info(f"New Goal: {self.goal_x}, {self.goal_y}, {self.goal_theta}")
            self.NEW_GOAL = True
        self.last_goal = [self.goal_x, self.goal_y, self.goal_theta]

        # Get Current Edges
        self.current_edges = self.edge_follower.current_edges

        return
    
    def timer_callback(self):
        """Main loop of the bug1 planner"""
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
    node = BugPlanner("Bug1")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
