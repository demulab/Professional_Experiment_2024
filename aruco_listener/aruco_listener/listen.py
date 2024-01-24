import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from geometry_msgs.msg import PoseArray, Pose


class FrameListener(Node):
    def __init__(self):
        super().__init__('aruco_listener')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(0.1, self.on_timer)
        
        self.pose_pub = self.create_publisher(Pose, 'aruco_poses_from_world', 10)

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        to_frame_rel = 'world'
        from_frame_rel = 'aruco_marker0'

        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        pose = Pose()
        pose.position.x = t.transform.translation.x
        pose.position.y = t.transform.translation.y
        pose.position.z = t.transform.translation.z
        pose.orientation.x = t.transform.rotation.x
        pose.orientation.y = t.transform.rotation.y
        pose.orientation.z = t.transform.rotation.z
        pose.orientation.w = t.transform.rotation.w
        self.pose_pub.publish(pose)
        self.get_logger().info(f'{to_frame_rel} to {from_frame_rel} : pose : {pose}')
  
        

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
