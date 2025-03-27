# python3 src/waypoint_follower/waypoint_follower/path_follower.py /custom_odometry_topic

#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
import time

from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import Twist, PoseStamped

from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class PathFollower(Node):
    """
    A ROS2 node that uses a PD controller to follow a path (list of waypoints).
    Subscribes to /path for the target path and a specified odometry topic for the robot's current state,
    and publishes velocity commands to /cmd_vel.
    """

    # NOTE: CANNOT CHANGE
    def __init__(self, odometry_topic):
        super().__init__('path_follower')

        self.setup_parameters()

        # sum of derivate
        self.sum_derivate = 0
        self.start_time = 0
        self.end_time = 0

        # Path and waypoints
        self.path = None
        self.current_waypoint_index = 0

        # Previous errors (for derivative term)
        self.prev_error_x = 0.0
        self.prev_error_y = 0.0
        self.prev_error_theta = 0.0

        # Robot current state
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_orientation = 0.0
        self.is_odom_received = False

        # Thresholds
        self.waypoint_threshold = 0.1  # Distance threshold to consider a waypoint reached
        self.orientation_threshold = 0.05  # Orientation threshold for final alignment

        # Publisher to cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.cmd_vel_pub.publish(Twist())

        # Subscriber to odometry (with the specified topic)
        self.odom_sub = self.create_subscription(
            Odometry,
            odometry_topic,
            self.odom_callback,
            10
        )

        # Subscriber to path
        self.path_sub = self.create_subscription(
            Path,
            '/planner/path',
            self.path_callback,
            10
        )

        # Timer to periodically publish velocity commands
        timer_period = 0.1  # [s] -> 10 Hz
        self.timer = self.create_timer(timer_period, self.control_loop_callback)
        self.get_logger().info(f"Path Follower node started. Subscribing to odometry topic: {odometry_topic}")

        # Path visualization publishers
        self.path_pub = self.create_publisher(Path, '/visualized_path', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/path_markers', 10)

    # NOTE: CANNOT CHANGE
    def normalize_angle(self, angle):
            """
            Normalize an angle to the range [-pi, pi].
            """
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle     

    # NOTE: CANNOT CHANGE
    def odom_callback(self, msg: Odometry):
        """
        Extracts and stores the robot's current pose from the odometry.
        """
        self.is_odom_received = True
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Convert quaternion to yaw (theta)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_orientation = math.atan2(siny_cosp, cosy_cosp)

    # NOTE: CANNOT CHANGE
    def check_ending(self):
        if not self.is_odom_received or self.path is None:
            self.cmd_vel_pub.publish(Twist())
            return True

        # Check if all waypoints are reached
        if self.current_waypoint_index >= len(self.path):
            if self.end_time < 1e-3:
                self.end_time = time.time()
            self.get_logger().info(f"----- All waypoints reached. Stopping. ---- ")
            self.get_logger().info(f"Sum of error: {self.sum_derivate:.3f}, Cost time: {self.end_time - self.start_time:.3f}s.")
            self.cmd_vel_pub.publish(Twist())  # Stop the robot
            return True
        
        return False 

    # NOTE: STUDENTS ARE ALLOWED TO ADD THEIR OWN FUNCTIONS
    def customized_functions(self):
        pass

    # NOTE: CAN CHANGE
    def setup_parameters(self):
        # Maximum velocities
        self.max_linear_vel = 1.0  # meter
        self.max_angular_vel = 0.5 # rad

        # PD Controller Gains (tune as necessary)
        self.Kp_linear = 1.0
        self.Kd_linear = 0.1
        self.Kp_angular = 1.0
        self.Kd_angular = 0.1

    def visulize_path(self):
        """
        Publishes the smoothed path for visualization.
        """
        if self.smoothed_path is not None:
            path_msg = Path()
            path_msg.header.stamp = self.get_clock().now().to_msg()
            path_msg.header.frame_id = 'map'
            path_msg.poses = self.smoothed_path
            self.path_pub.publish(path_msg)
            self.get_logger().info(f"Published smoothed path with {len(self.smoothed_path)} points.")   
    
    def publish_path_markers(self):
        """
        Publishes markers (arrows and text) for each waypoint to RViz.
        """
        if self.path is None:
            return

        marker_array = MarkerArray()
        for idx, pose_stamped in enumerate(self.path):
            pos = pose_stamped.pose.position

            # Arrow marker
            arrow = Marker()
            arrow.header = pose_stamped.header
            arrow.ns = "waypoint_arrows"
            arrow.id = idx
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.pose = pose_stamped.pose
            arrow.scale.x = 0.6  # shaft length
            arrow.scale.y = 0.05
            arrow.scale.z = 0.05
            arrow.color = ColorRGBA(r=0.0, g=0.5, b=1.0, a=1.0)  # blue
            marker_array.markers.append(arrow)

            # Text marker
            text = Marker()
            text.header = pose_stamped.header
            text.ns = "waypoint_labels"
            text.id = 1000 + idx
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = pos.x
            text.pose.position.y = pos.y
            text.pose.position.z = 0.3  # Slightly above ground
            text.scale.z = 0.2  # Font size
            text.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)  # red
            text.text = str(idx)
            marker_array.markers.append(text)

        self.marker_pub.publish(marker_array)
        self.get_logger().info("Published waypoint markers.")


    # NOTE: CAN CHANGE
    def path_callback(self, msg: Path):
        """
        Updates the target path when a new message is received.
        """
        self.path = msg.poses  # List of PoseStamped messages
 
        self.current_waypoint_index = 0  # Reset to the first waypoint
        self.get_logger().info(f"Received new path with {len(self.path)} waypoints.")

        for i in range(len(self.path) - 1):
            current = self.path[i].pose.position
            next_ = self.path[i + 1].pose.position

            dx = next_.x - current.x
            dy = next_.y - current.y
            yaw = math.atan2(dy, dx)

            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            self.path[i].pose.orientation.z = qz
            self.path[i].pose.orientation.w = qw

        if len(self.path) >= 2:
            self.path[-1].pose.orientation = self.path[-2].pose.orientation

        self.start_time = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        self.end_time = 0

        self.visulize_path()
        self.publish_path_markers()

    # NOTE: CAN CHANGE
    def control_loop_callback(self):
        """
        Periodic control loop callback. Computes PD control and publishes velocity commands.
        """
        if self.check_ending():
            return

        # Get the current target waypoint
        target_pose = self.path[self.current_waypoint_index].pose
        x_target = target_pose.position.x
        y_target = target_pose.position.y

        # Convert quaternion to yaw (theta) for the target orientation
        q = target_pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        orientation_target = math.atan2(siny_cosp, cosy_cosp)

        # 1) Compute errors
        error_x = x_target - self.current_x
        error_y = y_target - self.current_y
        error_theta = self.normalize_angle(math.atan2(error_y, error_x) - self.current_orientation) # degree
        error_orientation = self.normalize_angle(orientation_target - self.current_orientation)

        # 2) Compute derivative of errors
        derivative_x = error_x - self.prev_error_x
        derivative_y = error_y - self.prev_error_y
        derivative_theta = error_theta - self.prev_error_theta
        self.sum_derivate += math.fabs(derivative_x) + math.fabs(derivative_y) + math.fabs(derivative_theta) / 180.0 * math.pi

        # 3) PD control for linear velocities (x, y)
        vx = self.Kp_linear * error_x + self.Kd_linear * derivative_x
        vy = self.Kp_linear * error_y + self.Kd_linear * derivative_y

        # 4) PD control for angular velocity
        vtheta = self.Kp_angular * error_theta + self.Kd_angular * derivative_theta

        # 5) Update previous error terms
        self.prev_error_x = error_x
        self.prev_error_y = error_y
        self.prev_error_theta = error_theta

        # 6) Publish velocity commands
        twist_msg = Twist()

        # Check if the robot has reached the current waypoint
        distance_to_waypoint = math.hypot(error_x, error_y)
        if distance_to_waypoint < self.waypoint_threshold:
            self.current_waypoint_index += 1  # Move to the next waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_index - 1}. Moving to the next one.")
        else:
            # Move toward the current waypoint
            if abs(error_theta) > 1.0 and distance_to_waypoint > self.waypoint_threshold:
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Rotating before moving forward")
            else:
                twist_msg.linear.x = min(math.hypot(vx, vy), self.max_linear_vel)
                twist_msg.angular.z = min(vtheta, self.max_angular_vel)
                self.get_logger().info("Moving forward")

        self.cmd_vel_pub.publish(twist_msg)

def main():
    import sys
    args = sys.argv 

    rclpy.init(args=args)

    # Specify the odometry topic (default is '/Odometry')
    odometry_topic = '/Odometry'  # Default value
    node = PathFollower(odometry_topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt detected, shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
