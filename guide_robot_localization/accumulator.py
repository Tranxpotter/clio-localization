#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np

class PointCloudAccumulator(Node):
    def __init__(self):
        super().__init__('pointcloud_accumulator')
        
        # PARAMETERS
        self.declare_parameter('accumulation_frames', 5) # 5 frames @ 10Hz = 0.5s
        self.target_frames = self.get_parameter('accumulation_frames').value
        
        # SUBSCRIBER (Input sparse cloud)
        # We use '/cloud_registered' because it's stable in World Frame
        self.sub = self.create_subscription(
            PointCloud2,
            '/cloud_registered',
            self.callback,
            10
        )
        
        # PUBLISHER (Output dense cloud)
        self.pub = self.create_publisher(PointCloud2, '/cloud_accumulated', 10)
        
        self.buffer = []
        self.get_logger().info(f"Accumulator started. Merging {self.target_frames} frames.")

    def callback(self, msg):
        # Add new cloud to buffer
        self.buffer.append(msg)
        
        # If buffer is full, merge and publish
        if len(self.buffer) >= self.target_frames:
            merged_msg = self.merge_clouds(self.buffer)
            self.pub.publish(merged_msg)
            # Remove oldest message (Sliding Window) 
            # OR clear buffer (Batch). Batch is safer for CPU.
            self.buffer = [] # Batch mode: clear buffer

    def merge_clouds(self, cloud_list):
        # Base the merged message on the latest message (for timestamp/frame_id)
        base_msg = cloud_list[-1]
        
        # Extract data from all clouds
        all_data = [cloud.data for cloud in cloud_list]
        
        # Concatenate the byte arrays (Efficient in Python)
        merged_data = b''.join(all_data)
        
        # Create new message
        new_msg = PointCloud2()
        new_msg.header = base_msg.header # Use latest timestamp/frame
        new_msg.height = 1
        # Width is sum of all widths
        new_msg.width = sum(c.width for c in cloud_list)
        new_msg.fields = base_msg.fields
        new_msg.is_bigendian = base_msg.is_bigendian
        new_msg.point_step = base_msg.point_step
        new_msg.row_step = new_msg.point_step * new_msg.width
        new_msg.is_dense = base_msg.is_dense
        new_msg.data = merged_data
        
        return new_msg

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()