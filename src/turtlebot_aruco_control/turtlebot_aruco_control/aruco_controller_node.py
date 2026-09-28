#!/usr/bin/env python3
"""
ArUco Controller Node for TurtleBot3
Detects ArUco markers and controls robot movement based on marker position
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoControllerNode(Node):
    """
    ROS2 node that detects ArUco markers and controls TurtleBot3 movement
    """
    
    def __init__(self):
        super().__init__('aruco_controller')
        
        # Declare parameters
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.0)
        self.declare_parameter('threshold', 20)
        self.declare_parameter('debug', True)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        
        # Get parameters
        aruco_dict_name = self.get_parameter('aruco_dict').value
        self.marker_id = self.get_parameter('marker_id').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.threshold = self.get_parameter('threshold').value
        self.debug = self.get_parameter('debug').value
        camera_topic = self.get_parameter('camera_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Initialize ArUco dictionary
        aruco_dict_type = getattr(cv2.aruco, aruco_dict_name)
        self.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
        self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        
        # Create subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            10
        )
        
        # State variables
        self.last_detection_time = self.get_clock().now()
        self.marker_detected = False
        
        self.get_logger().info('ArUco Controller Node started')
        self.get_logger().info(f'Listening to camera topic: {camera_topic}')
        self.get_logger().info(f'Publishing to velocity topic: {cmd_vel_topic}')
        self.get_logger().info(f'Looking for ArUco marker ID: {self.marker_id}')
        self.get_logger().info(f'ArUco dictionary: {aruco_dict_name}')
        self.get_logger().info(f'Linear speed: {self.linear_speed} m/s')
        self.get_logger().info(f'Threshold: {self.threshold} pixels')
    
    def image_callback(self, msg):
        """
        Callback function for camera images
        Detects ArUco markers and controls robot based on marker position
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Convert to grayscale for ArUco detection
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers
            corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)
            
            # Create velocity command
            twist = Twist()
            
            # Check if any markers were detected
            if ids is not None and len(ids) > 0:
                # Draw detected markers if debug mode is enabled
                if self.debug:
                    cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Look for our specific marker ID
                marker_found = False
                for i, marker_id in enumerate(ids):
                    if marker_id[0] == self.marker_id:
                        marker_found = True
                        
                        # Get marker corners
                        marker_corners = corners[i][0]
                        
                        # Calculate marker center
                        center_x = int(np.mean(marker_corners[:, 0]))
                        center_y = int(np.mean(marker_corners[:, 1]))
                        
                        # Get image center
                        image_height, image_width = cv_image.shape[:2]
                        image_center_y = image_height // 2
                        
                        # Calculate difference from center
                        diff_y = center_y - image_center_y
                        
                        # Draw marker center and image center line
                        if self.debug:
                            cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)
                            cv2.line(cv_image, (0, image_center_y), 
                                   (image_width, image_center_y), (255, 0, 0), 2)
                            
                            # Display position text
                            if abs(diff_y) <= self.threshold:
                                position_text = "CENTERED - STOPPED"
                                color = (0, 255, 255)  # Yellow
                            elif diff_y < -self.threshold:
                                position_text = "ABOVE CENTER - FORWARD"
                                color = (0, 255, 0)  # Green
                            else:
                                position_text = "BELOW CENTER - BACKWARD"
                                color = (0, 0, 255)  # Red
                            
                            cv2.putText(cv_image, position_text, (10, 30),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
                            cv2.putText(cv_image, f"Diff Y: {diff_y}", (10, 60),
                                      cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                        
                        # Control logic based on marker position
                        if abs(diff_y) <= self.threshold:
                            # Marker is centered - stop
                            twist.linear.x = 0.0
                            self.get_logger().info('Marker centered - STOP', 
                                                 throttle_duration_sec=1.0)
                        elif diff_y < -self.threshold:
                            # Marker is above center - move forward
                            twist.linear.x = self.linear_speed
                            twist.angular.z = self.angular_speed
                            self.get_logger().info(f'Marker above center - FORWARD at {self.linear_speed} m/s', 
                                                 throttle_duration_sec=1.0)
                        else:
                            # Marker is below center - move backward
                            twist.linear.x = -self.linear_speed
                            twist.angular.z = self.angular_speed
                            self.get_logger().info(f'Marker below center - BACKWARD at {self.linear_speed} m/s', 
                                                 throttle_duration_sec=1.0)
                        
                        # Update detection status
                        self.marker_detected = True
                        self.last_detection_time = self.get_clock().now()
                        break
                
                if not marker_found:
                    self.marker_detected = False
                    if self.debug:
                        cv2.putText(cv_image, "Target marker not found", (10, 30),
                                  cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # No markers detected
                self.marker_detected = False
                if self.debug:
                    cv2.putText(cv_image, "No markers detected", (10, 30),
                              cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Publish velocity command
            self.cmd_vel_pub.publish(twist)
            
            # Display image if debug mode is enabled
            if self.debug:
                cv2.imshow('ArUco Detection', cv_image)
                cv2.waitKey(1)
        
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    """
    Main function to initialize and run the ArUco controller node
    """
    rclpy.init(args=args)
    
    try:
        node = ArucoControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup
        if rclpy.ok():
            # Stop the robot before shutting down
            node.get_logger().info('Shutting down - stopping robot')
            twist = Twist()  # All zeros
            node.cmd_vel_pub.publish(twist)
            node.destroy_node()
            rclpy.shutdown()
        
        # Close OpenCV windows
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
