#!/usr/bin/env python3
"""
Odometry Transformer for LiDAR Mounting Correction
Subscribes to FAST-LIO odometry and applies LiDAR mounting transformations
to correct the camera_init -> body transform for the actual robot base
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from scipy.spatial.transform import Rotation as R


class OdometryTransformer(Node):
    def __init__(self):
        super().__init__('odometry_transformer')
        
        # Parameters for LiDAR mounting position (same as in launch file)
        self.declare_parameter('x_offset', 0.0)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.0)
        self.declare_parameter('pitch_angle_deg', 240.0)  # 240 degrees
        self.declare_parameter('yaw_angle_deg', 180.0)    # 180 degrees z-rotation
        
        # Get parameters
        self.x_offset = self.get_parameter('x_offset').value
        self.y_offset = self.get_parameter('y_offset').value
        self.z_offset = self.get_parameter('z_offset').value
        pitch_deg = self.get_parameter('pitch_angle_deg').value
        yaw_deg = self.get_parameter('yaw_angle_deg').value
        
        # Convert to radians
        pitch_rad = math.radians(pitch_deg)
        yaw_rad = math.radians(yaw_deg)
        
        # Create the LiDAR mounting transformation matrix
        # This represents base_footprint -> livox_frame transform
        self.lidar_mount_transform = self.create_transform_matrix(
            [self.x_offset, self.y_offset, self.z_offset],
            [0, pitch_rad, yaw_rad]  # roll, pitch, yaw
        )
        
        # Inverse transform (livox_frame -> base_footprint)
        self.lidar_mount_transform_inv = np.linalg.inv(self.lidar_mount_transform)
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to FAST-LIO odometry
        self.odom_sub = self.create_subscription(
            Odometry,
            '/Odometry',  # FAST-LIO default odometry topic
            self.odometry_callback,
            10
        )
        
        # Publish corrected odometry on standard nav stack topic
        self.corrected_odom_pub = self.create_publisher(
            Odometry,
            '/odom',  # Standard topic that SLAM Toolbox expects
            10
        )
        
        self.get_logger().info(f'Odometry transformer initialized')
        self.get_logger().info(f'LiDAR offset: x={self.x_offset}, y={self.y_offset}, z={self.z_offset}')
        self.get_logger().info(f'LiDAR rotation: pitch={pitch_deg}°, yaw={yaw_deg}°')
    
    def create_transform_matrix(self, translation, rotation_rpy):
        """Create 4x4 transformation matrix from translation and RPY rotation"""
        # Create rotation matrix from roll, pitch, yaw
        r = R.from_euler('xyz', rotation_rpy)
        rotation_matrix = r.as_matrix()
        
        # Create 4x4 homogeneous transformation matrix
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = translation
        
        return transform
    
    def matrix_to_pose_and_quaternion(self, transform_matrix):
        """Convert 4x4 transformation matrix to position and quaternion"""
        # Extract translation
        translation = transform_matrix[:3, 3]
        
        # Extract rotation matrix and convert to quaternion
        rotation_matrix = transform_matrix[:3, :3]
        r = R.from_matrix(rotation_matrix)
        quaternion = r.as_quat()  # Returns [x, y, z, w]
        
        return translation, quaternion
    
    def odometry_callback(self, msg):
        try:
            # Extract pose from odometry message
            pos = msg.pose.pose.position
            orient = msg.pose.pose.orientation
            
            # Convert pose to 4x4 transformation matrix (camera_init -> body from FAST-LIO)
            position = np.array([pos.x, pos.y, pos.z])
            quaternion = np.array([orient.x, orient.y, orient.z, orient.w])
            
            # Create transformation matrix from FAST-LIO pose
            r = R.from_quat(quaternion)
            fast_lio_transform = np.eye(4)
            fast_lio_transform[:3, :3] = r.as_matrix()
            fast_lio_transform[:3, 3] = position
            
            # Apply LiDAR mounting correction
            # The goal is to transform from camera_init -> body to camera_init -> base_footprint
            # Since FAST-LIO gives us camera_init -> body, and we want camera_init -> base_footprint
            # We need: T_camera_base = T_camera_body * T_body_base
            # Where T_body_base = inverse of T_base_body (LiDAR mount transform)
            
            corrected_transform = fast_lio_transform @ self.lidar_mount_transform_inv
            
            # Convert back to position and quaternion
            corrected_position, corrected_quaternion = self.matrix_to_pose_and_quaternion(corrected_transform)
            
            # Create corrected odometry message
            corrected_odom = Odometry()
            corrected_odom.header = msg.header
            corrected_odom.header.frame_id = 'odom'  # Standard odom frame
            corrected_odom.child_frame_id = 'base_footprint'
            
            # Set corrected pose
            corrected_odom.pose.pose.position.x = corrected_position[0]
            corrected_odom.pose.pose.position.y = corrected_position[1]
            corrected_odom.pose.pose.position.z = corrected_position[2]
            corrected_odom.pose.pose.orientation.x = corrected_quaternion[0]
            corrected_odom.pose.pose.orientation.y = corrected_quaternion[1]
            corrected_odom.pose.pose.orientation.z = corrected_quaternion[2]
            corrected_odom.pose.pose.orientation.w = corrected_quaternion[3]
            
            # Copy velocity (assuming it's in the body frame and needs similar correction)
            # For simplicity, we'll keep the original velocity for now
            corrected_odom.twist = msg.twist
            
            # Copy covariance
            corrected_odom.pose.covariance = msg.pose.covariance
            corrected_odom.twist.covariance = msg.twist.covariance
            
            # Publish corrected odometry
            self.corrected_odom_pub.publish(corrected_odom)
            
            # Broadcast the corrected transform
            self.broadcast_transform(corrected_odom)
            
            # Debug output (every 50 messages to avoid spam)
            if hasattr(self, '_msg_count'):
                self._msg_count += 1
            else:
                self._msg_count = 1
                
            if self._msg_count % 50 == 0:
                self.get_logger().info(
                    f'Original pos: [{pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}] '
                    f'-> Corrected pos: [{corrected_position[0]:.2f}, {corrected_position[1]:.2f}, {corrected_position[2]:.2f}]'
                )
            
        except Exception as e:
            self.get_logger().error(f'Failed to transform odometry: {str(e)}')
    
    def broadcast_transform(self, odom_msg):
        """Broadcast the corrected transform"""
        try:
            transform = TransformStamped()
            transform.header = odom_msg.header
            transform.child_frame_id = odom_msg.child_frame_id
            
            transform.transform.translation.x = odom_msg.pose.pose.position.x
            transform.transform.translation.y = odom_msg.pose.pose.position.y
            transform.transform.translation.z = odom_msg.pose.pose.position.z
            
            transform.transform.rotation.x = odom_msg.pose.pose.orientation.x
            transform.transform.rotation.y = odom_msg.pose.pose.orientation.y
            transform.transform.rotation.z = odom_msg.pose.pose.orientation.z
            transform.transform.rotation.w = odom_msg.pose.pose.orientation.w
            
            self.tf_broadcaster.sendTransform(transform)
            
        except Exception as e:
            self.get_logger().error(f'Failed to broadcast transform: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    odometry_transformer = OdometryTransformer()
    rclpy.spin(odometry_transformer)
    odometry_transformer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
