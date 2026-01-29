'''
GoalPoseRemapper Node
This node reads nav2 goal pose topic and call FASTLIO2_ROS2 localizer topic 

Parameters
-----------
verbose: `bool`
    Log what the node is doing
'''


'''Note for FASTLIO2_ROS2 interfaces:

Relocalize: 
string pcd_path
float32 x
float32 y
float32 z
float32 yaw
float32 pitch
float32 roll
---
bool success
string message

IsValid:
int32 code
---
bool valid
'''

from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import rclpy
from rclpy.node import Node
import numpy as np
import math
from interface.srv import Relocalize, IsValid # Service of FASTLIO2_ROS2 /localizer/relocalize and /localizer/relocalize_check
from geometry_msgs.msg import PoseStamped # This is the msg type of /goal_pose

class GoalPoseRemapper(Node):
    def __init__(self):
        super().__init__('goal_pose_remapper')
        
        # Parameters
        self.declare_parameter('verbose', True)
        self.verbose = self.get_parameter('verbose').get_parameter_value().bool_value
    



        # Subscriber
        self.subscription = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_pose_callback,
            10)
        self.relocalize_client = self.create_client(Relocalize, "/localizer/relocalize")
        

    def goal_pose_callback(self, msg):
        ...
            

def main(args=None):
    rclpy.init(args=args)
    goal_pose_remapper = GoalPoseRemapper()
    rclpy.spin(goal_pose_remapper)
    goal_pose_remapper.destroy_node()
    rclpy.shutdown()
