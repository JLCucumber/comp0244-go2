#!/usr/bin/env python3

# ros2 topic pub /waypoint geometry_msgs/Pose2D "{x: -0.8, y: 1.1, theta: 0.0}" -r 1
# ros2 topic pub /waypoint geometry_msgs/Pose2D "{x: 0.5, y: 1.1, theta: 3.14}" -r 1
# ros2 run cw1_team_2 advanced_edge_follower 


import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from edge_follower.edge_follower_node import EdgeFollowerNode
from tf_transformations import quaternion_from_euler
from ..utils.utils import save_to_csv
import csv
import os

# import debugpy



class AdvancedEdgeFollowerNodes(EdgeFollowerNode):
    def __init__(self):
        super().__init__()

        # 
        self.reached_line_end = False

        # Feature: no need for cross product (seems doesn't work well)
        self.moving_forward = "counter-clockwise" # "clockwise" or "counter-clockwise"
        
        # Constants
        self.SAFETY_MARGIN = 0.6  # meters
        self.INCREMENT_DISTANCE = 0.5 # meters 0.7
        self.UPDATE_RATE = 0.1  # seconds 0.5
        self.OVERSHOOT = 0.3  # meters
        self.POINT_THRESHOLD = 0.8 # meters
        
        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []  # List of (start_point, end_point) tuples
        self.last_waypoint = None
        self.last_closest_point = None
        
        # Publishers
        self.waypoint_pub = self.create_publisher(Pose2D, 'waypoint', 10)
        self.waypoint_marker_pub = self.create_publisher(Marker, 'current_waypoint', 10)
        self.edge_marker_pub = self.create_publisher(Marker, 'detected_edges', 10)
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            'Odometry',
            self.odom_callback,
            10)
            
        self.line_sub = self.create_subscription(
            Marker,
            'local_map_lines',
            self.line_callback,
            10)
            
        # Timer
        self.timer = self.create_timer(self.UPDATE_RATE, self.timer_callback)
        
        self.get_logger().info('Edge Follower node initialized')

    def odom_callback(self, msg):
        """Update current robot pose (x,y,theta) from odometry (in odom frame)"""
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    def transform_to_camera_init(self, point):
        """Transform a point from livox to camera_init frame, which is our fixed world frame"""
        # Rotation matrix
        c = math.cos(self.current_orientation)
        s = math.sin(self.current_orientation)
        
        # Transform point
        x = point[0] * c - point[1] * s + self.current_x
        y = point[0] * s + point[1] * c + self.current_y
        theta = point[2] + self.current_orientation
        
        # self.get_logger().info(f"Current Pose: {self.current_x:.2f}, {self.current_y:.2f}, {self.current_orientation:.2f}")
        # self.get_logger().info(f"Goal Pose: {x:.2f}, {y:.2f}, {theta:.2f}")
        # self.get_logger().info(f"Distance Diff: {np.linalg.norm(np.array([x, y]) - np.array([self.current_x, self.current_y])):.2f}")

        return np.array([x, y, theta])

    def transform_to_base_link(self, point):
        """Transform a point from camera_init to livox frame"""
        # Translate to origin
        dx = point[0] - self.current_x
        dy = point[1] - self.current_y
        
        # Rotation matrix inverse
        c = math.cos(-self.current_orientation)
        s = math.sin(-self.current_orientation)
        
        # Transform point
        x = dx * c - dy * s
        y = dx * s + dy * c
        
        return np.array([x, y])
    


    def line_callback(self, msg):
        """Process incoming line segments"""

        # Print msg ID and timestamp
        # self.get_logger().info(f"Received Marker ID: {msg.id}, Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # save to csv
        path = "/workspace/comp0244-go2/src/cw1_team_2/cw1_team_2/data/edge_follower.csv"
        save_to_csv(msg, path)
        

        if len(msg.points) < 2:
            return
        # self.get_logger().info(f"Received Marker ID: {msg.id}, Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}")

        # Convert line points to numpy arrays
        points = [np.array([point.x, point.y]) for point in msg.points]
        
        # Create edges by connecting adjacent points
        for i in range(len(points) - 1):
            edge = (points[i], points[i+1]) # (start_point, end_point)
            if len(self.current_edges) > 0:
                if (tuple(edge[0]), tuple(edge[1])) in [(tuple(e[0]), tuple(e[1])) for e in self.current_edges]:
                    continue
            self.current_edges.append(edge)

    def find_next_waypoint(self):
        """Find closest edge and calculate next waypoint"""
        
        if not self.current_edges:
            return None
        
        # Feature: end-line "lag" - move in original direction for 1 more step
        if self.reached_line_end == True:
            self.get_logger().info("!!! Reached the end of the line")
            self.get_logger().info("!!! Holding the last waypoint for 1 step")
            self.reached_line_end = False
            return self.last_waypoint
        
        # Robot is at origin in livox frame
        robot_pos = np.array([0.0, 0.0])
        
        # First find the closest point on any edge segment
        closest_edge = None
        closest_point = None
        min_distance = float('inf')
        closest_edge_index = 0
        
        for i, edge in enumerate(self.current_edges):
            start_point, end_point = edge
            
            # Calculate edge vector
            edge_vector = end_point - start_point
            edge_length = np.linalg.norm(edge_vector)
            
            if edge_length < 0.01:  # Skip very short edges
                continue
                
            # Normalize edge vector
            edge_direction = edge_vector / edge_length

            # Vector from start point to robot
            to_robot = robot_pos - start_point
            
            # Project robot position onto edge line
            projection = np.dot(to_robot, edge_direction)
            projection = max(0, min(edge_length, projection))
            point_on_edge = start_point + projection * edge_direction

            # distance between current point and the closest point in last time
            if self.last_closest_point is not None:
                self.get_logger().info(f"last_closest_point is None. Initializing...")
                distance_to_lasttime_closest_point = np.linalg.norm(self.last_closest_point - point_on_edge)  
            else:
                distance_to_lasttime_closest_point = 0.0

            if distance_to_lasttime_closest_point < self.POINT_THRESHOLD:        # condition 1
                distance_to_robot = np.linalg.norm(point_on_edge - robot_pos) 
                if distance_to_robot < min_distance:                             # condition 2
                    min_distance = distance_to_robot
                    closest_edge = edge
                    closest_point = point_on_edge
                    closest_edge_index = i

        
        if closest_edge is None:
            return None
        else:
            self.closest_edge_point = closest_point     # Update current closest point (red dot)

        # # Store the closest point (red dot)
        
        
        # if closest_point_dist > self.POINT_THRESHOLD:
        #     self.get_logger().info(f"!!! Abnormal Closest Point Detected!!! (distance: {closest_point_dist:.2f})")
        #     # still use the last closest point
            
        #     self.closest_edge = self.last_closest_edge
        #     self.closest_edge_index = self.last_closest_edge_index
        # else:
        #     self.closest_edge_point = closest_point
        #     self.closest_edge = closest_edge
        #     self.closest_edge_index = closest_edge_index

        # Now move clockwise along the continuous edge by INCREMENT_DISTANCE
        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        
        # Vector from closest point to robot
        to_robot = robot_pos - closest_point
        
        # Determine cw direction using cross product (NO, CROSS PRODUCT IS USELESS)
        moving_forward = self.moving_forward
        # cross_z = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        # # moving_forward = cross_z > 0  
        # moving_forward = cross_z < 0  # Feature: change to counter-clockwise
        
        # Move along edges to find increment point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point
        
        if moving_forward == "clockwise":

            while increment_left > 0 and current_index < len(self.current_edges):

                current_edge = self.current_edges[current_index]

                start, end = current_edge
                remaining_distance = np.linalg.norm(end - current_point)
                self.get_logger().info("---------- MOVING CLOCKWISE ----------")
                self.get_logger().info(f"current edge {current_index} out of {len(self.current_edges)}")
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    self.get_logger().info("\033[92m### Keep moving on the same edge ###\033[0m")
                    self.get_logger().info(f"\033[92mincrement_left: {increment_left:.2f}, remaining_distance: {remaining_distance:.2f}\033[0m")
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point + edge_direction * increment_left
                    
                    break
                else:
                    # Move to next edge
                    self.get_logger().info(f"\033[94m### move to next edge ###\033[0m")
                    self.get_logger().info(f"\033[94mcurrent edge {current_index+1} out of {len(self.current_edges)}\033[0m")
                    self.get_logger().info(f"\033[94mincrement_left: {increment_left:.2f}, remaining_distance: {remaining_distance:.2f}\033[0m")

                    increment_left -= np.linalg.norm(end - start)  # Fxcking bug here
                    current_index += 1    # Yeah I know, we don't need to increase current point here, don't worry
                    # once we increase the index, the waypoint will be calculated based on the new edge

                    if current_index >= len(self.current_edges):
                        # If we reach the end of the whole line, stay at the last point
                        self.reached_line_end = True

                        current_index = len(self.current_edges) - 1
                        current_point = self.current_edges[current_index][1]
                        self.get_logger().info(f"\033[94mSetting waypoint as the end of the line ({current_index}out of{len(self.current_edges)})\033[0m")
                        break
        elif moving_forward == "counter-clockwise":
            
            while increment_left > 0 and current_index >= 0:
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(current_point - start)
                self.get_logger().info("---------- MOVING COUNTER-CLOCKWISE ----------")
                self.get_logger().info(f"current edge {current_index} out of {len(self.current_edges)}")
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    self.get_logger().info("\033[92m### Keep moving on the same edge ###\033[0m")
                    self.get_logger().info(f"\033[92mincrement_left: {increment_left:.2f}, remaining_distance: {remaining_distance:.2f}\033[0m")
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point - edge_direction * increment_left
                    break
                else:
                    # Move to previous edge
                    self.get_logger().info(f"\033[94m### move to previous edge ###\033[0m")
                    self.get_logger().info(f"\033[94mcurrent edge {current_index+1} out of {len(self.current_edges)}\033[0m")
                    self.get_logger().info(f"\033[94mincrement_left: {increment_left:.2f}, remaining_distance: {remaining_distance:.2f}\033[0m")

                    increment_left -= np.linalg.norm(end - start)
                    current_index -= 1  
                    # Yeah I know, we don't need to increase current point here, don't worry
                    # once we increase the index, the waypoint will be calculated based on the new edge

                    if current_index < 0:
                        # If we reach the start, stay at the first point
                        self.reached_line_end = True
                        current_index = 0
                        current_point = self.current_edges[current_index][0] #- edge_direction * self.OVERSHOOT

                        self.get_logger().info(f"\033[94mSetting waypoint as the start of the line ({current_index}out of{len(self.current_edges)})\033[0m")
                        break
        else:
            self.get_logger().info("!!! Invalid moving direction !!!")
            raise ValueError("Invalid moving direction")

        # Store the incremented point (green dot)
        if self.reached_line_end == True:
            # Feature: end-line "lag" => assure robot find new lines 
            self.incremented_point = current_point - edge_direction * self.OVERSHOOT # edge_direction is clockwise, so set it negative
        else:
            print("setting increment point")
            self.incremented_point = current_point
        
        # Get the edge direction at the incremented point
        print(f"current_index: {current_index}")
        current_edge = self.current_edges[current_index]
        start, end = current_edge

        edge_direction = (end - start) / np.linalg.norm(end - start)
        print(f"start: {start}, end: {end}")
        print(f"edge_direction: {edge_direction}")  # is a vector

        # Calculate angle with respect to (0,0)
        edge_angle = math.atan2(edge_direction[1], edge_direction[0])  # Feature: add edge direction to waypoint angle
        print(f"edge_angle: {edge_angle}")
        
        # Calculate perpendicular vector (rotate edge_direction 90 degrees)
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        
        # Check which side the robot is on
        to_robot = robot_pos - current_point
        if np.dot(perpendicular, to_robot) < 0:
            perpendicular = -perpendicular  # Flip if needed to point toward robot's side
        
        # Calculate waypoint by projecting perpendicular to the edge
        # # Feature: add edge direction to waypoint (x,y) ==> (x,y,theta)
        waypoint = self.incremented_point + perpendicular * self.SAFETY_MARGIN
        
        if moving_forward == "clockwise":
            waypoint = np.array([waypoint[0], waypoint[1], edge_angle]) 
        elif moving_forward == "counter-clockwise":
            waypoint = np.array([waypoint[0], waypoint[1], edge_angle + math.pi]) # Feature: rotate 180 degrees in edge direction
        else:
            self.get_logger().info("Invalid moving direction !")
            raise ValueError("Invalid moving direction")

        # self.get_logger().info(f"Waypoint: {waypoint}")
        return waypoint

    def publish_visualizations(self, current_waypoint):
        """Publish visualization markers"""
        # Current waypoint
        if current_waypoint is not None:
            point_marker = Marker()
            point_marker.header.frame_id = "livox"
            point_marker.header.stamp = self.get_clock().now().to_msg()
            point_marker.type = Marker.ARROW  # SPHERE
            point_marker.action = Marker.ADD
            point_marker.id = 0
            
            point_marker.pose.position.x = float(current_waypoint[0])
            point_marker.pose.position.y = float(current_waypoint[1])
            quaternion = quaternion_from_euler(0, 0, float(current_waypoint[2]))
            point_marker.pose.orientation.x = quaternion[0]
            point_marker.pose.orientation.y = quaternion[1]
            point_marker.pose.orientation.z = quaternion[2]
            point_marker.pose.orientation.w = quaternion[3]
            
            point_marker.scale = Vector3(x=0.2, y=0.1, z=0.1)  # Arrow

            # point_marker.scale = Vector3(x=0.2, y=0.2, z=0.2)
            point_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
            self.waypoint_marker_pub.publish(point_marker)
            
            # Closest point on edge (red)
            if hasattr(self, 'closest_edge_point'):
                edge_point_marker = Marker()
                edge_point_marker.header.frame_id = "livox"
                edge_point_marker.header.stamp = self.get_clock().now().to_msg()
                edge_point_marker.type = Marker.SPHERE
                edge_point_marker.action = Marker.ADD
                edge_point_marker.id = 1
                
                edge_point_marker.pose.position.x = float(self.closest_edge_point[0])
                edge_point_marker.pose.position.y = float(self.closest_edge_point[1])
                edge_point_marker.pose.position.z = 0.0
                
                edge_point_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
                edge_point_marker.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
                self.waypoint_marker_pub.publish(edge_point_marker)
                
            # Incremented point on edge (green)
            if hasattr(self, 'incremented_point'):
                inc_point_marker = Marker()
                inc_point_marker.header.frame_id = "livox"
                inc_point_marker.header.stamp = self.get_clock().now().to_msg()
                inc_point_marker.type = Marker.SPHERE
                inc_point_marker.action = Marker.ADD
                inc_point_marker.id = 2
                
                inc_point_marker.pose.position.x = float(self.incremented_point[0])
                inc_point_marker.pose.position.y = float(self.incremented_point[1])
                inc_point_marker.pose.position.z = 0.0
                
                inc_point_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
                inc_point_marker.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
                self.waypoint_marker_pub.publish(inc_point_marker)        
        
        # Debug Usage: Draw the very start and the very end of the edge_marker
        very_start = self.current_edges[0][0]
        very_end = self.current_edges[-1][1]

        # new marker for very start and very end
        very_start_marker = Marker()
        very_start_marker.header.frame_id = "livox"
        very_start_marker.header.stamp = self.get_clock().now().to_msg()
        very_start_marker.type = Marker.CUBE
        very_start_marker.action = Marker.ADD
        very_start_marker.id = 3        
        very_start_marker.pose.position.x = float(very_start[0])
        very_start_marker.pose.position.y = float(very_start[1])
        very_start_marker.pose.position.z = 0.0
        very_start_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        very_start_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        self.waypoint_marker_pub.publish(very_start_marker)

        very_end_marker = Marker()
        very_end_marker.header.frame_id = "livox"
        very_end_marker.header.stamp = self.get_clock().now().to_msg()
        very_end_marker.type = Marker.CUBE
        very_end_marker.action = Marker.ADD
        very_end_marker.id = 4
        very_end_marker.pose.position.x = float(very_end[0])
        very_end_marker.pose.position.y = float(very_end[1])
        very_end_marker.pose.position.z = 0.0
        very_end_marker.scale = Vector3(x=0.1, y=0.1, z=0.1)
        very_end_marker.color = ColorRGBA(r=0.0, g=1.0, b=1.0, a=1.0)
        self.waypoint_marker_pub.publish(very_end_marker)
        
        # Detected edges
        edge_marker = Marker()
        edge_marker.header.frame_id = "livox"
        edge_marker.header.stamp = self.get_clock().now().to_msg()
        edge_marker.type = Marker.LINE_LIST
        edge_marker.action = Marker.ADD
        edge_marker.id = 0
        
        edge_marker.scale.x = 0.05
        edge_marker.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=1.0)

        for start_point, end_point in self.current_edges:
            edge_marker.points.append(Point(x=float(start_point[0]), y=float(start_point[1]), z=0.0))
            edge_marker.points.append(Point(x=float(end_point[0]), y=float(end_point[1]), z=0.0))
            
        self.edge_marker_pub.publish(edge_marker)

        

        self.current_edges = []  # Clear edges

    def timer_callback(self):
        """Main control loop"""
        if not self.is_odom_received:
            return
        
        # Store last waypoint
        self.last_waypoint = getattr(self, "current_waypoint", None)   # waypoint dimension: [x, y, theta]
        self.last_closest_point = getattr(self, "closest_edge_point", None)

        # Calculate next waypoint
        next_waypoint = self.find_next_waypoint()
        if next_waypoint is None:
            print("!!!! No next waypoint !!!!")
            return

        # Mitigate waypoints to avoid sharp turns (only if we have a last waypoint)
        if self.reached_line_end and next_waypoint is not None:
            next_waypoint = self.mitigate_waypoints(next_waypoint)
        
        # Update current waypoint
        self.current_waypoint = next_waypoint

        # Publish waypoint if valid
        if next_waypoint is not None:
            waypoint_msg = Pose2D()
            waypoint_camera_init = self.transform_to_camera_init(next_waypoint)
            waypoint_msg.x = float(waypoint_camera_init[0])
            waypoint_msg.y = float(waypoint_camera_init[1])
            waypoint_msg.theta = float(waypoint_camera_init[2])  #  the orientation of the edge
            
            self.waypoint_pub.publish(waypoint_msg)
            
        # Publish visualizations
        self.publish_visualizations(next_waypoint)

    def mitigate_waypoints(self, current_waypoint):
        """Feature: Mitigate waypoints to avoid sharp turns"""
        if self.last_waypoint is None:
            return current_waypoint

        # Calculate distance to last waypoint
        distance_diff = np.linalg.norm(current_waypoint - self.last_waypoint)
        
        # # If distance is too short, keep the last waypoint
        # if distance < 0.05:
        #     return self.last_waypoint
        
        # If distance is too long, average with last waypoint
        if self.reached_line_end: # and angle_diff > x: distance_diff > 1.0 and 
            temp = 0.6 * current_waypoint + 0.4 * self.last_waypoint
            return temp

        return current_waypoint

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedEdgeFollowerNodes()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':


    main()
