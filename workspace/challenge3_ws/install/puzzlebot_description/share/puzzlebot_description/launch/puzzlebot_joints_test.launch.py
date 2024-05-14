from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, Group
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declarar argumentos
    robot1_arg = DeclareLaunchArgument('robot1', default_value='puzzlebot_hacker_ed')
    robot2_arg = DeclareLaunchArgument('robot2', default_value='puzzlebot_jetson_ed')
    robot3_arg = DeclareLaunchArgument('robot3', default_value='puzzlebot_jetson_lidar_ed')

    # Transformaciones estáticas
    world_odom1_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '1', '0', '0', '0', '0', 'world', LaunchConfiguration('robot1') + '/base_link'],
        name='world_odom1_tf'
    )
    world_odom2_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', LaunchConfiguration('robot2') + '/base_link'],
        name='world_odom2_tf'
    )
    world_odom3_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '-1', '0', '0', '0', '0', 'world', LaunchConfiguration('robot3') + '/base_link'],
        name='world_odom3_tf'
    )

    # Cargar modelos Xacro
    # Debes asegurarte de que los xacro se procesen correctamente, aquí solo se define cómo se podría hacer
    robot1_description = PathJoinSubstitution([
        FindPackageShare('puzzlebot_description'), 'urdf', 'puzzlebot_hacker_ed_v1_3.xacro'
    ])

    # Grupos para cada robot
    robot1_group = Group(
        actions=[
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=LaunchConfiguration('robot1'),
                parameters=[{'robot_description': Command(['xacro ', robot1_description])}],
                name='puzzlebot_state_pub'
            ),
            Node(
                package='joint_state_publisher_gui',
                executable='joint_state_publisher_gui',
                name='joint_state_publisher_gui'
            )
        ]
    )

    # Lanzar RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('puzzlebot_description'), 'rviz', 'puzzlebot_test.rviz'
        ])]
    )

    return LaunchDescription([
        robot1_arg,
        robot2_arg,
        robot3_arg,
        world_odom1_tf,
        world_odom2_tf,
        world_odom3_tf,
        robot1_group,
        rviz_node
    ])

