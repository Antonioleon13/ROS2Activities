#!/usr/bin/env python3
import rclpy
import numpy as np
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node 
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
import time


w = 0.0
v = 0.0

def cb_cmd_vel(msg):
    global v, w
    v = msg.linear.x
    w = msg.angular.z

def diferential_drive_model(_v, _w, theta):
    vel_x = _v*np.cos(theta)
    vel_y = _v*np.sin(theta)
    vel_ang = _w

    return vel_x, vel_y, vel_ang    

def solver(vel_x, vel_y, vel_ang, x, y, theta):
    dt = 1/100
    x += vel_x*dt
    y += vel_y*dt
    theta += vel_ang*dt

    return x, y, theta

def transform(_v, _w):
    l = 0.19
    r = 0.05
    wr = (2*_v + _w*l)/(2*r)
    wl = (2*_v - _w*l)/(2*r)

    return wr, wl


def main():
    rclpy.init()
    node = rclpy.create_node("puzzlebot_kinematic_model")
    rate = node.create_rate(100)

    x = 0.0
    y = 0.0
    theta = 0.0
   

    # Publisher and subscribers
    pub_pose = node.create_publisher(PoseStamped, '/pose',10)
    pub_wl = node.create_publisher(Float32, '/wl', 10)
    pub_wr = node.create_publisher(Float32, '/wr', 10)
    sub_cmd_vel = node.create_subscription(Twist, '/cmd_vel', cb_cmd_vel, 10)


    print("The kinematic model is Running")

    pose_msg = PoseStamped()
    wl_msg = Float32()
    wr_msg = Float32()

    
    try:
        while rclpy.ok():
            vel_x, vel_y, vel_ang = diferential_drive_model(v, w, theta)
            x, y, theta = solver(vel_x, vel_y, vel_ang, x, y, theta)
            wr, wl = transform(v, w)
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = theta

            
            wr_msg.data = wr
            wl_msg.data = wl

            pub_pose.publish(pose_msg)
            pub_wl.publish(wl_msg)
            pub_wr.publish(wr_msg)

            time.sleep(1/100)
            rclpy.spin_once(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    
