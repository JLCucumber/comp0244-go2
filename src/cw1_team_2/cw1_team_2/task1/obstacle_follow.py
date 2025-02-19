#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose2D, Point, Vector3
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from edge_follower.edge_follower_node import EdgeFollowerNode

class AdvancedEdgeFollowerNodes(EdgeFollowerNode):
    def __init__(self):
        super().__init__()
        
        # Constants
        self.SAFETY_MARGIN = 1.0  # meters
        self.INCREMENT_DISTANCE = 0.7 # meters
        self.UPDATE_RATE = 0.5  # seconds

        # State variables
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False
        self.current_edges = []  # List of (start_point, end_point) tuples
        
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
            

        self.get_logger().info('Advanced Edge Follower Node has been initialized')


    def find_next_waypoint(self):
        """Find closest edge and calculate next waypoint"""
        if not self.current_edges:
            return None
            
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
            
            # Calculate distance to edge
            distance = np.linalg.norm(point_on_edge - robot_pos)
            
            if distance < min_distance:
                min_distance = distance
                closest_edge = edge
                closest_point = point_on_edge
                closest_edge_index = i
        
        if closest_edge is None:
            return None

        # Store the closest point (red dot)
        self.closest_edge_point = closest_point

        # Now move clockwise along the continuous edge by INCREMENT_DISTANCE
        start_point, end_point = closest_edge
        edge_vector = end_point - start_point
        edge_direction = edge_vector / np.linalg.norm(edge_vector)
        
        # Vector from closest point to robot
        to_robot = robot_pos - closest_point
        
        # Determine cw direction using cross product
        cross_z = edge_direction[0] * to_robot[1] - edge_direction[1] * to_robot[0]
        moving_forward = cross_z > 0
        
        # Move along edges to find increment point
        current_index = closest_edge_index
        increment_left = self.INCREMENT_DISTANCE
        current_point = closest_point
        
        if moving_forward:
            while increment_left > 0 and current_index < len(self.current_edges):
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(end - current_point)
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point + edge_direction * increment_left
                    break
                else:
                    # Move to next edge
                    increment_left -= remaining_distance
                    current_index += 1
                    if current_index >= len(self.current_edges):
                        # If we reach the end, stay at the last point
                        current_index = len(self.current_edges) - 1
                        current_point = self.current_edges[current_index][1]
                        break
        else:
            while increment_left > 0 and current_index >= 0:
                current_edge = self.current_edges[current_index]
                start, end = current_edge
                remaining_distance = np.linalg.norm(current_point - start)
                
                if increment_left <= remaining_distance:
                    # We can reach our point on this edge
                    edge_direction = (end - start) / np.linalg.norm(end - start)
                    current_point = current_point - edge_direction * increment_left
                    break
                else:
                    # Move to previous edge
                    increment_left -= remaining_distance
                    current_index -= 1
                    if current_index < 0:
                        # If we reach the start, stay at the first point
                        current_index = 0
                        current_point = self.current_edges[current_index][0]
                        break

        # Store the incremented point (green dot)
        self.incremented_point = current_point
        
        # Get the edge direction at the incremented point
        current_edge = self.current_edges[current_index]
        start, end = current_edge
        edge_direction = (end - start) / np.linalg.norm(end - start)
        
        # Calculate perpendicular vector (rotate edge_direction 90 degrees)
        perpendicular = np.array([-edge_direction[1], edge_direction[0]])
        
        # Check which side the robot is on
        to_robot = robot_pos - current_point
        if np.dot(perpendicular, to_robot) < 0:
            perpendicular = -perpendicular  # Flip if needed to point toward robot's side
            
        # Calculate waypoint by projecting perpendicular to the edge
        waypoint = current_point + perpendicular * self.SAFETY_MARGIN
        
        return waypoint
    
    # # 也可以添加新的方法
    # def new_feature(self):
    #     """新增功能"""
    #     self.get_logger().info("New feature activated!")


    def main(args=None):
        rclpy.init(args=args)
        node = EdgeFollowerNode()
        
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            node.get_logger().info('Keyboard interrupt, shutting down...')
        finally:
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
