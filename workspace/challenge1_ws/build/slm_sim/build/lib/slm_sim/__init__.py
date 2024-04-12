#!/usr/bin/env python3
import rclpy
import numpy as numpy
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from rclpy.node import Node 

#variables
k = 0.01
m = 0.75
l = 0.36
a = l/2
g = 9.8
Tau = 0.0
dt = 1/100
J = (4/3)*m*(a**2)

def update_pos(x1, x2):
    x1 += x2*dt
    x2_dot = (1/J) * (Tau - m*g*a * np.cos(x1) - k*x2)
    x2 += x2_dot*dt

    return x1, x2

def main():
    rclpy.init()
    node = Node("SLM_sim")
    rate = node.create_rate(100)
    pub = node.create_publisher(JointState, '/joint_states',10)
    msg = JointState()
    print("The SLM sim is Running")

    x1 = 0.0
    x2 = 0.0
    try:
        while rclpy.ok:
            x1, x2 = update_pos(x1, x2)
            theta = wrap_to_Pi(x1)

            msg.header.stamp = node.get_clock().now().to_msg()
            msg.name = ["joint2"]
            msg.position = [theta]
            msg.velocity = [x2]

            pub.publish(msg) 
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def wrap_to_Pi(theta):
    result = np.fmod((theta + np.pi), (2*np.pi))
    if(result < 0):
        result += 2 * np.pi
    
    return result - np.pi

if __name__ == '__main__':
    main()
    
