'''
TfDensifier Node
This node listens to a given tf transform and republishes it at a regular interval

Parameters
-----------
parent_frame: `str`
    The parent frame of the tf transform to listen to. Default: map
child_frame: `str`
    The child frame of the tf transform to listen to. Default: camera_init
hertz: `float`
    How fast should the tf be published. Default: 10.0
verbose: `bool`
    Log tf densifier
'''


from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

class TfDensifier(Node):
    def __init__(self):
        super().__init__('tf_densifier')
        
        # Parameters
        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'camera_init')
        self.declare_parameter('hertz', 10.0)
        self.declare_parameter('verbose', True)

        self.parent_frame = self.get_parameter('parent_frame').get_parameter_value().string_value
        self.child_frame = self.get_parameter('child_frame').get_parameter_value().string_value
        self.hertz = self.get_parameter('hertz').get_parameter_value().double_value
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value

        if self.verbose:
            self.get_logger().info(f"Loaded parameters: {self.parent_frame=} {self.child_frame=} {self.hertz=}")
    
        # TF Buffer and Listener with longer cache time
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.timer = self.create_timer(1/self.hertz, self.on_timer)

    def on_timer(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.child_frame,
                self.parent_frame,
                Time())
        except Exception as ex:
            if self.verbose:
                self.get_logger().info(
                    f'Could not transform {self.parent_frame} to {self.child_frame}: {ex}')
            return
        

        # edit transform timestamp
        t.header.stamp = self.get_clock().now().to_msg()

        self.tf_broadcaster.sendTransform(t)

        if self.verbose:
            self.get_logger().info(f"Published transform from {self.parent_frame} to {self.child_frame} with sec={t.header.stamp.sec} nanosec={t.header.stamp.nanosec}")

            

def main(args=None):
    rclpy.init(args=args)
    tf_height_remover = TfDensifier()
    try:
        rclpy.spin(tf_height_remover)
    except KeyboardInterrupt as e:
        pass
    tf_height_remover.destroy_node()
    rclpy.shutdown()
