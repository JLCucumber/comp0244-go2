import rclpy
import math
import numpy as np
from rclpy.node import Node
from cw1_team_2.task1.cw1_edge_follower import AdvancedEdgeFollowerNodes
from geometry_msgs.msg import Pose2D
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry

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
        self.UPDATE_RATE = 0.1  # [s]
        self.WAYPOINT_DISTANCE = 0.2  # [m]
        self.WAYPOINT_TOLERANCE = 0.1 # [m]
        self.OBSTACLE_DISTANCE = 1.0  # [m]

        # Variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.current_waypoint = None
        self.last_waypoint = None
        self.current_edges = []
        self.NEW_GOAL = False
        self.last_goal = [self.current_x, self.current_y, self.current_theta]

        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)

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
        for i in range(len(edges)):
            for j in range(len(edges1)):
                if self.do_intersect(edges[i][0], edges[i][1], edges1[j][0], edges1[j][1]):
                    return True
        return False

    def do_intersect(self, p1, q1, p2, q2):
        """Check if two line segments (p1, q1) and (p2, q2) intersect"""
        def orientation(p, q, r):
            """Find the orientation of the triplet (p, q, r)"""
            val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
            if val == 0:
                return 0
            return 1 if val > 0 else 2

        def on_segment(p, q, r):
            if min(p[0], r[0]) <= q[0] <= max(p[0], r[0]) and min(p[1], r[1]) <= q[1] <= max(p[1], r[1]):
                return True
            return False

        o1 = orientation(p1, q1, p2)
        o2 = orientation(p1, q1, q2)
        o3 = orientation(p2, q2, p1)
        o4 = orientation(p2, q2, q1)

        if o1 != o2 and o3 != o4:
            return True
        if o1 == 0 and on_segment(p1, p2, q1):
            return True
        if o2 == 0 and on_segment(p1, q2, q1):
            return True
        if o3 == 0 and on_segment(p2, p1, q2):
            return True
        if o4 == 0 and on_segment(p2, q1, q2):
            return True
        return False
    
    def is_obstacle_detected(self, theta_livox, current_edges):
        # Check if any edge in the way (within a rectangular area of distance OBSTACLE_DISTANCE towards the goal and width of the robot)
        # If there is an edge in the way, follow the edge
        rectange_width = 0.3
        rect = np.zeros((4,2))
        rect[0] = [1/2*rectange_width*math.sin(theta_livox), 1/2*rectange_width*math.cos(theta_livox)]
        rect[1] = [rect[0][0] + self.OBSTACLE_DISTANCE*math.sin(theta_livox + np.pi/2), rect[0][1] + self.OBSTACLE_DISTANCE*math.cos(theta_livox + np.pi/2)]
        rect[2] = [rect[1][0] + rectange_width*math.sin(theta_livox + np.pi), rect[1][1] + rectange_width*math.cos(theta_livox + np.pi)]
        rect[3] = [rect[2][0] + self.OBSTACLE_DISTANCE*math.sin(theta_livox - np.pi/2), rect[2][1] + self.OBSTACLE_DISTANCE*math.cos(theta_livox - np.pi/2)]        
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
        Count = 0
        # Check if there is an obstacle in the way
        while True:
            if not self.is_obstacle_detected(theta_livox, self.current_edges):
                Count += 1
                if Count == 100 or not self.timer.is_canceled():
                    break
            else:
                Count = 0
            # pause the timer
            self.timer.cancel()
            rclpy.spin_once(self.edge_follower)
            self.update_data()
            goal_livox = self.edge_follower.transform_to_base_link([self.goal_x, self.goal_y])
            theta_livox = math.atan2(goal_livox[1], goal_livox[0])
            
        if self.timer.is_canceled():
            self.get_logger().info("Obstacle Cleared") # TODO: Unexpected Triggering Timer Reset Which means the timer is not paused
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
