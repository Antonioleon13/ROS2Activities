import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Definir los argumentos de lanzamiento
    declare_robot_arg = DeclareLaunchArgument(
        'robot', default_value='puzzlebot',
        description='Name of the robot')

    # config = os.path.join(
    #   get_package_share_directory('puzzlebot_control'),
    #   'config',
    #   'puzzlebot_joint_states.yaml'
    #   )
    # config2 = os.path.join(
    #   get_package_share_directory('puzzlebot_control'),
    #   'config',
    #   'puzzlebot_diff_control.yaml'
    #   )

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        output='screen'
    )

    # Spawn controller manager
    controller_spawner_joint_state = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=['joint_state_controller', '--controller-manager', "/controller_manager"],
        output='screen',
        parameters=[
                PathJoinSubstitution([FindPackageShare('puzzlebot_control'), 'config', 'puzzlebot_joint_states.yaml'])
                ]
    )

    controller_spawner_base_control = Node(
        package='controller_manager',
        executable='spawner',
        name='controller_spawner',
        arguments=["base_controller", "--controller-manager", "/controller_manager"],
        output='screen',
        parameters=[
                PathJoinSubstitution([FindPackageShare('puzzlebot_control'), 'config', 'puzzlebot_diff_control.yaml']),
                ]
    )

    # Puzzlebot wheel velocity node
    wheel_vel_node = Node(
        package='puzzlebot_control',
        executable='wheel_Vel',
        name='wheel_vel'
    )

    # Return the launch description
    return LaunchDescription([
        # declare_robot_arg,
        controller_manager_node,
        controller_spawner_joint_state,
        controller_spawner_base_control,
        wheel_vel_node
    ])

