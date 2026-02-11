'''
PCSyncedStaticOdomPublisher Node
This node publishes a static odometry for tricking the localizer into doing the things I want it to do 

Parameters
-----------
input_topic: `str`
    The PointCloud2 topic to sync the static odometry with. Default: /Laser_map
output_topic: `str`
    The topic to publish the static odometry to. Default: /static_odom
parent_frame: `str`
    The parent frame for the odometry. Default: /map
child_frame: `str`
    The child frame frame for the odometry. Default: /static_odom
verbose: `bool`
    Logging. Default: False
'''


import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

class PCSyncedStaticOdomPublisher(Node):
    def __init__(self):
        super().__init__('pc_synced_static_odom_publisher')
        
        # Parameters
        self.declare_parameter('input_topic', '/Laser_map')
        self.declare_parameter('output_topic', '/static_odom')
        self.declare_parameter('parent_frame', '/map')
        self.declare_parameter('child_frame', '/static_odom')
        self.declare_parameter('verbose', False)

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        # Subscriber and Publisher
        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)

        self.subscriber = self.create_subscription(PointCloud2, self.input_topic, self.on_input_cb, 10)

    def on_input_cb(self, msg:PointCloud2):
        self.publish_odom(msg.header.stamp)

    def publish_odom(self, stamp):
        odom_msg = Odometry()
        odom_msg.header.frame_id = self.parent_frame
        odom_msg.header.stamp = stamp
        odom_msg.child_frame_id = self.child_frame

        odom_msg.pose.pose.position.x = 0.0
        odom_msg.pose.pose.position.y = 0.0
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.w = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0

        if self.verbose:
            self.get_logger().info(f"Publishing static odom {self.parent_frame}->{self.child_frame}")

        self.publisher.publish(odom_msg)
            
            

def main(args=None):
    rclpy.init(args=args)
    static_odom_publisher = PCSyncedStaticOdomPublisher()
    try:
        rclpy.spin(static_odom_publisher)
    except KeyboardInterrupt as e:
        pass
    static_odom_publisher.destroy_node()
    rclpy.shutdown()
