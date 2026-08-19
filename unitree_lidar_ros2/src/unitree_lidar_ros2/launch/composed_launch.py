# **********************************************************************
#  Copyright (c) 2020-2024, Unitree Robotics.Co.Ltd. All rights reserved.
# **********************************************************************
"""Run the lidar driver as a composable node inside a component container.

Loading the driver and whatever consumes the point cloud - a SLAM front end, a
filter, a recorder - into the same container lets them exchange each scan through
intra-process delivery instead of serialising ~160 kB per cloud over the loopback
interface. Add those nodes to `composable_node_descriptions` below.

    ros2 launch unitree_lidar_ros2 composed_launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
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
            'container_name',
            default_value='unilidar_container',
            description='Name of the component container.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Take time from /clock. Forces unified arrival timestamp mode.',
        ),
    ]

    container = ComposableNodeContainer(
        name=LaunchConfiguration('container_name'),
        namespace='',
        package='rclcpp_components',
        # component_container_mt runs a multi-threaded executor, so a slow
        # subscriber callback cannot hold up the rest of the container.
        executable='component_container_mt',
        output='screen',
        composable_node_descriptions=[
            ComposableNode(
                package='unitree_lidar_ros2',
                plugin='unitree_lidar_ros2::UnitreeLidarNode',
                name='unitree_lidar_ros2_node',
                parameters=[
                    LaunchConfiguration('config_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')},
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
    )

    return LaunchDescription(arguments + [container])
