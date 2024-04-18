#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node 
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import time


w = 0.0
v = 0.0
wl = 0.0
wr = 0.0



def wl_cb(msg):
    global wl
    wl = msg.data

def wr_cb(msg):
    global wr
    wr = msg.data

def differential_drive_model(v, w, theta):
    vel_x = v*np.cos(theta)
    vel_y = v*np.sin(theta)
    vel_ang = w

    return vel_x, vel_y, vel_ang    

def solver(vel_x, vel_y, vel_ang, x, y, theta):
    dt = 1/100
    x += vel_x*dt
    y += vel_y*dt
    theta += vel_ang*dt

    return x, y, theta

def transform(_wl, _wr):
    l = 0.19
    r = 0.05
    v = r * (_wr + _wl) / 2
    w = r * (_wr - _wl) / l

    return v, w


def main():
    rclpy.init()
    node = rclpy.create_node("puzzlebot_kinematic_model")
    rate = node.create_rate(100)

    x = 0.0
    y = 0.0
    theta = 0.0
   

    # Publisher and subscribers
    pub_odom = node.create_publisher(Odometry, '/odom',10)
    sub_wl = node.create_subscription(Float32, '/wl', wl_cb, 10)
    sub_wr = node.create_subscription(Float32, '/wr', wr_cb, 10)


    print("The localization node is Running")

    odom_msg = Odometry()

    try:
        while rclpy.ok():
            v, w = transform(wl, wr)
            vel_x, vel_y, vel_ang = differential_drive_model(v, w, theta)
            x, y, theta = solver(vel_x, vel_y, vel_ang, x, y, theta)

            odom_msg.header.stamp = node.get_clock().now().to_msg()
            odom_msg.header.frame_id = 'odom'

            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            odom_msg.pose.pose.orientation.w = theta

            odom_msg.twist.twist.linear.x = v
            odom_msg.twist.twist.angular.z = vel_ang

            pub_odom.publish(odom_msg)

            time.sleep(1/100)
            
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
