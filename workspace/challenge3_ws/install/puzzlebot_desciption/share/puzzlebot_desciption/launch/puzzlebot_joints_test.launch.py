from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare the robot names as arguments
    robot1 = DeclareLaunchArgument('robot1', default_value='puzzlebot_hacker_ed')
    robot2 = DeclareLaunchArgument('robot2', default_value='puzzlebot_jetson_ed')
    robot3 = DeclareLaunchArgument('robot3', default_value='puzzlebot_jetson_lidar_ed')

    # Path to the xacro files
    pkg_puzzlebot_description = os.path.join(get_package_share_directory('puzzlebot_description'), 'urdf')

    # Nodes for static transforms
    tf_world_odom1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_odom1_tf',
        arguments=['0', '1', '0', '0', '0', '0', 'world', LaunchConfiguration('robot1') + '/base_link']
    )

    tf_world_odom2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_odom2_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'world', LaunchConfiguration('robot2') + '/base_link']
    )

    tf_world_odom3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_odom3_tf',
        arguments=['0', '-1', '0', '0', '0', '0', 'world', LaunchConfiguration('robot3') + '/base_link']
    )

    # Nodes for robot state publisher and joint state publisher GUI for each robot
    groups = []
    for robot in ['robot1', 'robot2', 'robot3']:
        robot_description_path = os.path.join(pkg_puzzlebot_description, f'puzzlebot_{robot}_v1.xacro')

        robot_state_publisher = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=LaunchConfiguration(robot),
            parameters=[{'robot_description': Command(['xacro ', robot_description_path])}]
        )

        joint_state_publisher_gui = Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            namespace=LaunchConfiguration(robot)
        )

        groups.append(GroupAction([
            robot_state_publisher,
            joint_state_publisher_gui
        ]))

    # Node for RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', os.path.join(pkg_puzzlebot_description, 'rviz', 'puzzlebot_test.rviz')]
    )

    return LaunchDescription([
        robot1,
        robot2,
        robot3,
        tf_world_odom1,
        tf_world_odom2,
        tf_world_odom3,
        *groups,
        rviz_node
    ])
