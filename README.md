# ROS2 Humble Simulation Packages Repository

Welcome to the ROS2 Humble Simulation Packages Repository! This repository includes a variety of ROS2 simulation packages designed for different purposes. Each package is housed in its own workspace and demonstrates specific aspects of robotics simulation using ROS2 Humble.

## Overview

ROS 2 (Robot Operating System 2) is a set of software libraries and tools for building robot applications. Simulation is a crucial part of robotics development, enabling developers to test algorithms, behaviors, and interactions in a virtual environment before deploying them onto physical robots.

This repository hosts various simulation packages covering different aspects of ROS2, including:

- Robot kinematics and dynamics simulation
- Sensor simulation (e.g., LiDAR, camera, IMU)
- Environment simulation (e.g., maps, obstacles)
- Behavior simulation (e.g., navigation, manipulation)

## Workspaces and Simulations

### 1. Pendulum Simulation

**Workspace:** `challenge1_ws./slm_sim`

This workspace contains a simulation of a pendulum using RViz. It demonstrates the dynamics of a simple pendulum, showcasing how to model and simulate basic physical systems in ROS2.

### 2. Mobile Robot Simulation in RViz

**Workspace:** `challenge2_ws/localization`

This workspace features a simulation of a mobile robot in RViz. The simulation calculates the dynamic model of a two-wheeled robot and visualizes its movements. This package is ideal for understanding robot kinematics and the implementation of dynamic models in ROS2.

### 3. Mobile Robot Simulation in Gazebo

**Workspace:** `challenge3_ws`

This workspace includes a more advanced simulation of the same mobile robot in Gazebo. It integrates a LiDAR sensor and places the robot in various obstacle-filled worlds, allowing for the development and testing of navigation algorithms and obstacle avoidance behaviors.

## Getting Started

Each workspace contains its own README file with detailed instructions on installation, setup, and usage. Please refer to the specific README files within each workspace for more information.

## Requirements

- ROS 2 Humble
- Gazebo
- RViz
