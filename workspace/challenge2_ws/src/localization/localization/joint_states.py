#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from std_msgs.msg import Float64
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time 

class JointStatesPublisher(Node):
    def __init__(self):
        super().__init__('joint_states')

        self.wr = Float64()
        self.wl = Float64()
        self.wr = 0.0
        self.wl = 0.0
        self.v = 0.0
        self.w = 0.0
        self.l = 0.19
        self.r = 0.05

        self.joint_msg = JointState()


        print("Joint node is running")
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)
        self.start_time = self.get_clock().now()
        timer_period = 1/10
        self.timer = self.create_timer(timer_period, self.update_joint_state)

    def update_joint_state(self):
        self.joint_msg.header.stamp = self.get_clock().now().to_msg()
        self.joint_msg.name = ["right_wheel", "left_wheel"]
        self.joint_msg.velocity = [self.wr, self.wl]
        self.pub_joint_states.publish(self.joint_msg)
    
    def odom_cb(self, msg):
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z
        self.transform

    def transform(self):
        self.wr = (2*self.v + self.w*self.l)/(2*self.r)
        self.wl = (2*self.v - self.w*self.l)/(2*self.r)


def main():
    rclpy.init()
    joint_state_node = JointStatesPublisher()
    rclpy.spin(joint_state_node)
    joint_state_node.destroy_node()
    rclpy.shutdown()

        