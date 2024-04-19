#!/usr/bin/env python3
import math
import rclpy
import numpy as np
from std_msgs.msg import Float32
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import time 

class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        self.x_d = 0.0
        self.y_d = 0.0
        self.theta_d = 0.0

        self.x_act = 0.0
        self.y_act = 0.0
        self.theta_act = 0.0

        self.err_x = 0.0
        self.err_y = 0.0
        self.err_linear = 0.0
        self.err_angular = 0.0
        self.err_linear_prev = 0.0
        self.err_angular_prev = 0.0

        self.integral_err_linear = 0.0
        self.integral_err_angular = 0.0


        self.v_d = 0.0
        self.w_d = 0.0
        
        self.kp_linear = 0.6
        self.ki_linear = 0.0
        self.kd_linear = 0.0

        self.kp_angular = 1.8
        self.ki_angular = 0.0
        self.kd_angular = 0.0

        self.cmd_vel_msg = Twist()
        self.theta_d_msg = Float32()

        print("Controller is running")
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.sub_set_point = self.create_subscription(Pose, '/set_point', self.set_point_cb, 10)
        self.pub_cmd_vel = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pub_theta_d = self.create_publisher(Float32, '/theta_d', 10)
        self.start_time = self.get_clock().now()
        timer_period = 1/10
        self.timer = self.create_timer(timer_period, self.update_controller)

    def calculate_dt(self):
        self.current_time = self.start_time.to_msg()
        self.duration = self.get_clock().now() - self.start_time
        self.dt = self.duration.nanoseconds * 1e-9

    def calculate_error_linear(self):
        self.err_linear_prev = self.err_linear
        self.err_x = self.x_d - self.x_act
        self.err_y = self.y_d - self.y_act
        self.err_linear = np.sqrt(self.err_x**2 + self.err_y**2)

    def calculate_error_angular(self):
        self.err_angular_prev = self.err_angular
        self.theta_d = math.atan2(self.err_y, self.err_x)
        self.err_angular = self.theta_d - self.theta_act
        self.theta_d_msg.data = self.theta_d

    def calculate_errors(self):
        self.calculate_error_linear()
        self.calculate_error_angular()

    def linear_controller(self):
        proportional = self.kp_linear * self.err_linear

        self.integral_err_linear += self.err_linear * self.dt
        integral = self.ki_linear * self.integral_err_linear

        derivate = ((self.err_linear - self.err_linear_prev)/self.dt)*self.kd_linear

        self.cmd_vel_msg.linear.x = proportional + integral + derivate

    def angular_controller(self):
        proportional = self.kp_angular * self.err_angular

        self.integral_err_angular += self.err_angular * self.dt
        integral = self.ki_angular * self.integral_err_angular

        derivate = ((self.err_angular - self.err_angular_prev)/self.dt)*self.kd_angular

        self.cmd_vel_msg.angular.z = proportional + integral + derivate
        
    def controller(self):
        self.calculate_errors()
        self.linear_controller()
        self.angular_controller()
   
    
    def update_controller(self):
        
        self.calculate_dt()
        self.controller()
        self.pub_theta_d.publish(self.theta_d_msg)
        self.pub_cmd_vel.publish(self.cmd_vel_msg)
        self.start_time = self.get_clock().now()

    def odom_cb(self, msg):
        self.x_act = msg.pose.pose.position.x
        self.y_act = msg.pose.pose.position.y
        self.theta_act = msg.pose.pose.orientation.w




    def set_point_cb(self, msg):
        self.x_d = msg.position.x
        self.y_d = msg.position.y


def main():
    rclpy.init()
    controller_node = Controller()
    rclpy.spin(controller_node)
    controller_node.destroy_node()
    rclpy.shutdown()






    