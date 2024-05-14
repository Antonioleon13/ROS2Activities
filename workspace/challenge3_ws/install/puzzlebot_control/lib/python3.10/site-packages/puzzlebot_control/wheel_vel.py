#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState

class WheelVel(Node):
    def __init__(self):
        super().__init__('wheel_velocity')
        self.subscription = self.create_subscription(
            JointState, 'joint_states', self.callback, 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)

    def callback(self, data):
        try:
            wr_idx = data.name.index('wheel_right_joint')
            wl_idx = data.name.index('wheel_left_joint')
            # Publish messages
            self.wr_pub.publish(Float32(data=data.velocity[wr_idx]))
            self.wl_pub.publish(Float32(data=data.velocity[wl_idx]))
        except Exception as e:
            self.get_logger().warn('Error in callback: {}'.format(e))

def main(args=None):
    rclpy.init(args=args)
    wheel_vel = WheelVel()
    print("Wheel Speed Publishing...")
    rclpy.spin(wheel_vel)
    # Cleanup
    wheel_vel.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
