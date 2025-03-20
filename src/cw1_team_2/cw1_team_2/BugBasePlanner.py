# import rclpy
# import math
# import numpy as np
# from rclpy.node import Node
# from cw1_team_2.task1.cw1_edge_follower import AdvancedEdgeFollowerNodes
# from geometry_msgs.msg import Pose2D
# from visualization_msgs.msg import Marker
# from nav_msgs.msg import Odometry
# from cw1_team_2.utils.utils import is_intersected, get_rect_marker

# class BugBasePlanner(Node):
#     def __init__(self, name):
#         super().__init__(name)
#         self.get_logger().info(f"{name} Node Started")
#         self.edge_follower = AdvancedEdgeFollowerNodes()

#         # Declare parameters
#         self.declare_parameter("goal_x", 0.0)
#         self.declare_parameter("goal_y", 0.0)
#         self.declare_parameter("goal_theta", 0.0)

#         # Constants
#         self.UPDATE_RATE = 0.2  # [s]
#         self.WAYPOINT_DISTANCE = 0.2  # [m]
#         self.WAYPOINT_TOLERANCE = 0.1  # [m]
#         self.OBSTACLE_DISTANCE = 1.5  # [m]

#         # Variables
#         self.current_x = 0.0
#         self.current_y = 0.0
#         self.current_theta = 0.0
#         self.current_edges = []
#         self.NEW_GOAL = False
#         self.last_goal = [self.current_x, self.current_y, self.current_theta]

#         # ROS 2 Publisher
#         self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 1)
#         self.rect_pub = self.create_publisher(Marker, 'rectangle', 10)

#         # ROS 2 Subscribers
#         self.odom_sub = self.create_subscription(Odometry, 'Odometry', self.edge_follower.odom_callback, 10)
#         self.line_sub = self.create_subscription(Marker, 'local_map_lines', self.edge_follower.line_callback, 10)

#         # ROS 2 Timer
#         self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)

#         self.get_logger().info(f'{name} initialized')

#     def is_goal_reached(self):
#         """ Check if the robot has reached the goal"""
#         distance = math.sqrt((self.current_x - self.goal_x) ** 2 + (self.current_y - self.goal_y) ** 2)
#         return distance < self.WAYPOINT_TOLERANCE

#     def is_edge_crossed(self, edges, edges1):
#         """ Check if two edges are intersected """
#         for edge in edges:
#             for edge1 in edges1:
#                 if is_intersected(edge[0], edge[1], edge1[0], edge1[1]):
#                     return True
#         return False

#     def is_obstacle_detected(self, theta_livox, current_edges):
#         """ Check if the obstacle is detected """
#         rectange_width = 0.3
#         rect = np.zeros((4, 2))
#         rect[0] = [1 / 2 * rectange_width * math.cos(theta_livox + np.pi / 2), 
#                    1 / 2 * rectange_width * math.sin(theta_livox + np.pi / 2)]
#         rect[1] = [rect[0][0] + self.OBSTACLE_DISTANCE * math.cos(theta_livox), 
#                    rect[0][1] + self.OBSTACLE_DISTANCE * math.sin(theta_livox)]
#         rect[2] = [rect[1][0] + rectange_width * math.cos(theta_livox - np.pi / 2), 
#                    rect[1][1] + rectange_width * math.sin(theta_livox - np.pi / 2)]
#         rect[3] = [rect[2][0] + self.OBSTACLE_DISTANCE * math.cos(theta_livox + np.pi), 
#                    rect[2][1] + self.OBSTACLE_DISTANCE * math.sin(theta_livox + np.pi)]

#         rect_msg = get_rect_marker(rect, self.get_clock().now().to_msg())
#         self.rect_pub.publish(rect_msg)

#         rect_edges = [np.array([rect[i], rect[(i + 1) % 4]]) for i in range(len(rect))]
#         return self.is_edge_crossed(current_edges, rect_edges)

#     def update_data(self):
#         """ Update the robot's current position and goal """
#         self.current_x = self.edge_follower.current_x
#         self.current_y = self.edge_follower.current_y
#         self.current_theta = self.edge_follower.current_orientation

#         self.goal_x = self.get_parameter("goal_x").get_parameter_value().double_value
#         self.goal_y = self.get_parameter("goal_y").get_parameter_value().double_value
#         self.goal_theta = self.get_parameter("goal_theta").get_parameter_value().double_value

#         self.goal = [self.goal_x, self.goal_y, self.goal_theta]
#         self.distance_to_goal = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)

#         if self.goal != self.last_goal:
#             self.get_logger().info(f"New Goal: {self.goal_x}, {self.goal_y}, {self.goal_theta}")
#             self.NEW_GOAL = True
#         self.last_goal = self.goal
#         self.current_edges = self.edge_follower.current_edges

#     def timer_callback(self):
#         """ Timer callback function """
#         if not self.edge_follower.is_odom_received:
#             return

#         self.update_data()

#         if self.is_goal_reached():
#             self.get_logger().info("Goal Reached")
#             self.NEW_GOAL = False
#             return

#         if self.NEW_GOAL:
#             self.move_to_goal()

#     def move_to_goal(self):
#         """ Move the robot to the goal """
#         raise NotImplementedError("Subclasses should implement move_to_goal()")
