import math
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from turtlesim.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q


class FramePublisher(Node):

    def __init__(self):
        super().__init__('transform_frame_publisher')


        self.wl_pos = 0.0
        self.wr_pos = 0.0

        self.dt_wl = 0.0
        self.dt_wr = 0.0

        self.start_time_wl = self.get_clock().now()
        self.start_time_wr = self.get_clock().now()

        # Initialize the transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to a turtle{1}{2}/pose topic and call handle_turtle_pose
        # callback function on each message
        self.subscription_odom = self.create_subscription(Odometry,'/odom',
            self.handle_puzzlebot_pose, 1)
        self.subscription_wr = self.create_subscription(Float32,'/wr',
            self.handle_puzzlebot_wr, 1)
        self.subscription_wl = self.create_subscription(Float32,'/wl',
            self.handle_puzzlebot_wl, 1)
        self.subscription_odom  # prevent unused variable warning
        self.subscription_wl
        self.subscription_wr

    def calculate_dt_wl(self):
        self.current_time = self.start_time_wl.to_msg()
        self.duration = self.get_clock().now() - self.start_time_wl
        self.dt_wl = self.duration.nanoseconds * 1e-9

    def calculate_dt_wr(self):
        self.current_time = self.start_time_wr.to_msg()
        self.duration = self.get_clock().now() - self.start_time_wr
        self.dt_wr = self.duration.nanoseconds * 1e-9
        

    def handle_puzzlebot_pose(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, msg.pose.pose.orientation.w)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)

    def handle_puzzlebot_wl(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'wl_link'

        self.wl_pos += msg.data * self.dt_wl

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = self.wl_pos
        t.transform.translation.y = self.wl_pos
        t.transform.translation.z = self.wl_pos

        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message

        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        self.start_time_wl = self.get_clock().now()

    def handle_puzzlebot_wr(self, msg):
        t = TransformStamped()

        # Read message content and assign it to
        # corresponding tf variables
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = 'wr_link'

        # Turtle only exists in 2D, thus we get x and y translation
        # coordinates from the message and set the z coordinate to 0

        self.wr_pos += msg.data * self.dt_wr

        t.transform.translation.x = self.wl_pos
        t.transform.translation.y = self.wr_pos
        t.transform.translation.z = self.wl_pos


        # For the same reason, turtle can only rotate around one axis
        # and this why we set rotation in x and y to 0 and obtain
        # rotation in z axis from the message
        q = quaternion_from_euler(0, 0, 0)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        # Send the transformation
        self.tf_broadcaster.sendTransform(t)
        print(self.wr_pos)
        self.start_time_wr = self.get_clock().now()

def main():
    rclpy.init()
    node = FramePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()