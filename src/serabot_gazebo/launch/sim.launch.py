import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('serabot_gazebo')
    pkg_description = get_package_share_directory('serabot_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(pkg_description, 'urdf', 'serabot.urdf.xacro')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'bridge.yaml')
    rviz_config = os.path.join(pkg_gazebo, 'config', 'sim.rviz')

    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    yaw = LaunchConfiguration('yaw')
    rviz = LaunchConfiguration('rviz')

    declare_args = [
        DeclareLaunchArgument(
            'world', default_value=os.path.join(pkg_gazebo, 'worlds', 'greenhouse.sdf'),
            description='Gazebo world dosyası'),
        DeclareLaunchArgument('x', default_value='-4.0', description='Başlangıç x (m)'),
        DeclareLaunchArgument('y', default_value='0.7', description='Başlangıç y (m)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='Başlangıç yönü (rad)'),
        DeclareLaunchArgument('rviz', default_value='true', description='RViz açılsın mı'),
    ]

    # 1) Gazebo ve sera dünyası (-r: simülasyon oynatılarak başlar)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={
            'gz_args': ['-r ', world],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # 2) robot_state_publisher: xacro modelini /robot_description olarak yayınlar
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen',
    )

    # 3) Robotu /robot_description konusundan alıp Gazebo'ya yerleştirir
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'serabot',
            '-x', x, '-y', y, '-z', '0.02', '-Y', yaw,
        ],
        output='screen',
    )

    # 4) Gazebo <-> ROS 2 konu köprüsü
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_config}],
        output='screen',
    )

    # 5) Hazır config ile RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(rviz),
        output='screen',
    )

    return LaunchDescription(declare_args + [
        gazebo,
        robot_state_publisher,
        spawn,
        bridge,
        rviz_node,
    ])
