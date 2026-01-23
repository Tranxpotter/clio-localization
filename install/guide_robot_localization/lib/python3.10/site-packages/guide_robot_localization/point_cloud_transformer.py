from tf2_ros import TransformBroadcaster, Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs_py import point_cloud2
import math

class PointCloudTransformer(Node):
    def __init__(self):
        super().__init__('point_cloud_transformer')
        
        # Parameters
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/transformed_point_cloud')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('timeout', 1.0)  # seconds
    
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.target_frame = self.get_parameter('target_frame').value
        self.timeout = self.get_parameter('timeout').value  # seconds
    
        # TF Buffer and Listener with longer cache time
        self.tf_buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscriber and Publisher
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.point_cloud_callback,
            10)
        self.publisher = self.create_publisher(PointCloud2, self.output_topic, 10)

    def point_cloud_callback(self, msg):
        try:
            # Check if transform is available
            if not self.tf_buffer.can_transform(
                self.target_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            ):
                self.get_logger().warn(f'Transform from {msg.header.frame_id} to {self.target_frame} not available')
                return

            self.get_logger().info(f'Transforming point cloud from {msg.header.frame_id} to {self.target_frame}')
            transformed_cloud = self.transform_cloud_manual(msg)
            if transformed_cloud is not None:
                # Publish the transformed point cloud
                self.publisher.publish(transformed_cloud)

        except Exception as e:
            self.get_logger().error(f'Failed to transform point cloud: {str(e)}')

    def transform_cloud_manual(self, msg):
        """Manual transformation method with robust timing"""
        try:
            # Try to get transform with the exact timestamp first
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    msg.header.stamp,
                    rclpy.duration.Duration(seconds=self.timeout)
                )
            except Exception:
                # Fallback: use the latest available transform
                self.get_logger().warn('Using latest available transform instead of message timestamp')
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    msg.header.frame_id,
                    rclpy.time.Time()  # Latest available
                )
            
            # Read points from the message
            points_list = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            
            if not points_list:
                self.get_logger().warn('No valid points in point cloud')
                return None
            
            # Transform each point with full 6-DOF transformation
            transformed_points = []
            
            # Extract rotation quaternion
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w
            
            # Convert quaternion to rotation matrix
            rotation_matrix = self.quaternion_to_rotation_matrix(qx, qy, qz, qw)
            
            # Extract translation
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            tz = transform.transform.translation.z
            
            for point in points_list:
                # Apply rotation
                point_vec = np.array([point[0], point[1], point[2]])
                rotated_point = rotation_matrix @ point_vec
                
                # Apply translation
                x = rotated_point[0] + tx
                y = rotated_point[1] + ty
                z = rotated_point[2] + tz
                
                transformed_points.append([x, y, z])
            
            # Create new point cloud message
            header = msg.header
            header.frame_id = self.target_frame
            header.stamp = self.get_clock().now().to_msg()  # Use current time
            
            # Create the new point cloud
            new_cloud = point_cloud2.create_cloud_xyz32(header, transformed_points)
            
            return new_cloud
            
        except Exception as e:
            self.get_logger().error(f'Transform lookup failed: {str(e)}')
            return None
    
    def quaternion_to_rotation_matrix(self, qx, qy, qz, qw):
        """
        Convert a quaternion to a 3x3 rotation matrix.
        """
        # Normalize quaternion
        norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
        if norm == 0:
            return np.eye(3)
        
        qx /= norm
        qy /= norm
        qz /= norm
        qw /= norm
        
        # Rotation matrix from quaternion
        rotation_matrix = np.array([
            [1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qw*qz), 2*(qx*qz + qw*qy)],
            [2*(qx*qy + qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qw*qx)],
            [2*(qx*qz - qw*qy), 2*(qy*qz + qw*qx), 1 - 2*(qx*qx + qy*qy)]
        ])
        
        return rotation_matrix
            
            

def main(args=None):
    rclpy.init(args=args)
    point_cloud_transformer = PointCloudTransformer()
    rclpy.spin(point_cloud_transformer)
    point_cloud_transformer.destroy_node()
    rclpy.shutdown()
