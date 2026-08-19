# **********************************************************************
#  Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
# **********************************************************************
"""Bring up the Unitree L2 lidar driver, and rviz2 to look at it.

Every setting lives in config/unilidar_l2.yaml. Override the file, or individual
parameters, from the command line::

    ros2 launch unitree_lidar_ros2 launch.py
    ros2 launch unitree_lidar_ros2 launch.py rviz:=false
    ros2 launch unitree_lidar_ros2 launch.py config_file:=/path/to/my.yaml
    ros2 launch unitree_lidar_ros2 launch.py log_level:=debug
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('unitree_lidar_ros2')

    arguments = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([package_share, 'config', 'unilidar_l2.yaml']),
            description='Parameter file for the lidar driver.',
        ),
        DeclareLaunchArgument(
            'rviz',
            default_value='true',
            description='Also start rviz2.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([package_share, 'rviz', 'view.rviz']),
            description='rviz2 configuration file.',
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace to push the driver into.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Take time from /clock. Forces unified arrival timestamp mode.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logger level for the driver.',
        ),
    ]

    use_sim_time = LaunchConfiguration('use_sim_time')

    lidar_node = Node(
        package='unitree_lidar_ros2',
        executable='unitree_lidar_ros2_node',
        name='unitree_lidar_ros2_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        # Without this, log lines from the driver are buffered when the output is
        # not a terminal.
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': use_sim_time},
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': use_sim_time}],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(arguments + [lidar_node, rviz_node])
