#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time

class KinematicModel(Node):

    def __init__(self):
        super().__init__('kinematic_model')

        self.l = 0.19
        self.r = 0.05
        self.w = 0.0
        self.v = 0.0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_ang = 0.0
        self.wl = 0.0
        self.wr = 0.0
        self.wr_msg = Float32()
        self.wl_msg = Float32()
        self.pose_msg = PoseStamped()

        print("The kinematic model is Running")

        self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/pose',10)
        self.pub_wl = self.create_publisher(Float32, '/wl', 10)
        self.pub_wr = self.create_publisher(Float32, '/wr', 10)

        self.start_time = self.get_clock().now()
        timer_period = 1/10  # seconds
        self.timer = self.create_timer(timer_period, self.update_pose)

    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def update_pose(self):

        self.calculate_dt()
        self.diferential_drive_model()
        self.solver()
        self.transform()
        self.wr_msg.data = self.wr
        self.wl_msg.data = self.wl
        self.pose_msg.pose.position.x = self.x
        self.pose_msg.pose.position.y = self.y
        self.pose_msg.pose.position.z = self.theta

        self.pub_pose.publish(self.pose_msg)
        self.pub_wl.publish(self.wl_msg)
        self.pub_wr.publish(self.wr_msg)
        self.start_time = self.get_clock().now()

    def cb_cmd_vel(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def diferential_drive_model(self):
        self.vel_x = self.v*np.cos(self.theta)
        self.vel_y = self.v*np.sin(self.theta)
        self.vel_ang = self.w    

    def solver(self):
        self.x += self.vel_x*self.dt
        self.y += self.vel_y*self.dt
        self.theta += self.vel_ang*self.dt

    def transform(self):
        self.wr = (2*self.v + self.w*self.l)/(2*self.r)
        self.wl = (2*self.v - self.w*self.l)/(2*self.r)

def main():
    rclpy.init()
    kinematic_model = KinematicModel()
    rclpy.spin(kinematic_model)
    kinematic_model.destroy_node()
    rclpy.shutdown()

    
