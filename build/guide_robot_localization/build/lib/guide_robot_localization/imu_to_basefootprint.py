#!/usr/bin/env python3
"""
Dynamic TF Publisher for Tilted Lidar
Subscribes to /livox/imu and publishes dynamic transform from base_footprint to livox_frame
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class DynamicLidarTFPublisher(Node):
    def __init__(self):
        super().__init__('dynamic_lidar_tf_publisher')
        
        # Parameters
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('lidar_frame', 'livox_frame')
        self.declare_parameter('x_offset', 0.1)
        self.declare_parameter('y_offset', 0.0)
        self.declare_parameter('z_offset', 0.5)
        
        self.base_frame = self.get_parameter('base_frame').value
        self.lidar_frame = self.get_parameter('lidar_frame').value
        self.x_offset = self.get_parameter('x_offset').value
        self.y_offset = self.get_parameter('y_offset').value
        self.z_offset = self.get_parameter('z_offset').value
        
        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Subscribe to IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/livox/imu',
            self.imu_callback,
            10
        )
        
        self.get_logger().info(f'Publishing dynamic TF from {self.base_frame} to {self.lidar_frame}')
        self.get_logger().info(f'Position offset: x={self.x_offset}, y={self.y_offset}, z={self.z_offset}')
    
    def imu_callback(self, msg):
        """
        Callback for IMU messages
        Publishes transform with IMU orientation
        """
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.base_frame
        t.child_frame_id = self.lidar_frame
        
        # Translation (fixed mounting position)
        t.transform.translation.x = self.x_offset
        t.transform.translation.y = self.y_offset
        t.transform.translation.z = self.z_offset
        
        # Rotation (from IMU orientation)
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w
        
        # Broadcast transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicLidarTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()