#!/usr/bin/env python3
"""
Corrected Body Frame Publisher Node

This node:
1. Listens for camera_init → body transform from FAST-LIO
2. Applies a 60° counter-clockwise Y-axis rotation to correct for LiDAR mounting
3. Publishes the corrected transform as odom → corrected_body

The goal is to make the X and Y axes of the corrected_body frame parallel to the ground.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
import numpy as np
import math
from scipy.spatial.transform import Rotation as R
import tf2_geometry_msgs
from rclpy.time import Time


class CorrectedBodyFramePublisher(Node):
    def __init__(self):
        super().__init__('corrected_body_frame_publisher')
        
        # Parameters
        self.declare_parameter('source_frame', 'camera_init')
        self.declare_parameter('intermediate_frame', 'body')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('output_frame', 'corrected_body')
        self.declare_parameter('correction_angle_deg', 60.0)  # Counter-clockwise Y-axis rotation
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('tf_timeout', 1.0)
        
        # Get parameters
        self.source_frame = self.get_parameter('source_frame').value
        self.intermediate_frame = self.get_parameter('intermediate_frame').value
        self.target_frame = self.get_parameter('target_frame').value
        self.output_frame = self.get_parameter('output_frame').value
        correction_deg = self.get_parameter('correction_angle_deg').value
        publish_rate = self.get_parameter('publish_rate').value
        self.tf_timeout = self.get_parameter('tf_timeout').value
        
        # Create correction rotation (60° counter-clockwise around Y-axis)
        correction_rad = math.radians(correction_deg)
        self.correction_rotation = R.from_euler('y', correction_rad)
        
        # TF setup
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing
        timer_period = 1.0 / publish_rate  # seconds
        self.timer = self.create_timer(timer_period, self.publish_corrected_transform)
        
        # Error tracking
        self.last_error_time = None
        self.error_interval = 5.0  # Log errors every 5 seconds
        
        self.get_logger().info(f'Corrected Body Frame Publisher initialized')
        self.get_logger().info(f'Source: {self.source_frame} → Intermediate: {self.intermediate_frame}')
        self.get_logger().info(f'Publishing: {self.target_frame} → {self.output_frame}')
        self.get_logger().info(f'Correction: {correction_deg}° counter-clockwise around Y-axis')

    def publish_corrected_transform(self):
        """Main function to get FAST-LIO transform, apply correction, and publish"""
        try:
            # Get current time
            now = self.get_clock().now()
            
            # Try to get the transform from FAST-LIO
            transform = self.get_transform_with_fallback(
                self.source_frame, 
                self.intermediate_frame, 
                now
            )
            
            if transform is None:
                return
            
            # Apply the correction rotation
            corrected_transform = self.apply_correction_rotation(transform)
            
            # Publish the corrected transform
            corrected_transform.header.stamp = now.to_msg()
            corrected_transform.header.frame_id = self.target_frame
            corrected_transform.child_frame_id = self.output_frame
            
            self.tf_broadcaster.sendTransform(corrected_transform)
            
        except Exception as e:
            self.log_periodic_error(f"Error in publish_corrected_transform: {e}")

    def get_transform_with_fallback(self, source_frame, target_frame, timestamp):
        """Get transform with fallback to latest available if timestamp lookup fails"""
        try:
            # First try with the requested timestamp
            transform = self.tf_buffer.lookup_transform(
                source_frame,
                target_frame,
                timestamp,
                timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
            )
            return transform
            
        except Exception as e:
            try:
                # Fallback: get latest available transform
                transform = self.tf_buffer.lookup_transform(
                    source_frame,
                    target_frame,
                    Time(),  # Latest available
                    timeout=rclpy.duration.Duration(seconds=self.tf_timeout)
                )
                return transform
                
            except Exception as e2:
                self.log_periodic_error(
                    f"Failed to get transform {source_frame} → {target_frame}: {e2}"
                )
                return None

    def apply_correction_rotation(self, original_transform):
        """Apply the Y-axis correction rotation to the transform"""
        # Extract original rotation
        orig_quat = [
            original_transform.transform.rotation.x,
            original_transform.transform.rotation.y,
            original_transform.transform.rotation.z,
            original_transform.transform.rotation.w
        ]
        orig_rotation = R.from_quat(orig_quat)
        
        # Apply correction: R_corrected = R_correction * R_original
        corrected_rotation = self.correction_rotation * orig_rotation
        corrected_quat = corrected_rotation.as_quat()
        
        # Create corrected transform
        corrected_transform = TransformStamped()
        corrected_transform.header = original_transform.header
        
        # Keep the same translation
        corrected_transform.transform.translation = original_transform.transform.translation
        
        # Apply corrected rotation
        corrected_transform.transform.rotation.x = corrected_quat[0]
        corrected_transform.transform.rotation.y = corrected_quat[1]
        corrected_transform.transform.rotation.z = corrected_quat[2]
        corrected_transform.transform.rotation.w = corrected_quat[3]
        
        return corrected_transform

    def log_periodic_error(self, message):
        """Log errors periodically to avoid spam"""
        now = self.get_clock().now().nanoseconds / 1e9
        if (self.last_error_time is None or 
            now - self.last_error_time > self.error_interval):
            self.get_logger().warn(message)
            self.last_error_time = now


def main(args=None):
    rclpy.init(args=args)
    node = CorrectedBodyFramePublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
