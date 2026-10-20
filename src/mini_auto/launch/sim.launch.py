"""実機なしで全体を動かす: sim_robot が ros2can + 機体 + LiDAR の代わりをする。

RViz の「2D Goal Pose」でゴールを置くとナビが走る。
D456 の取り付けは cam_x / cam_y / cam_z / cam_pitch (sim_robot は TF から読んで深度画像を作る)。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('mini_auto')
    robot_params = os.path.join(pkg, 'config', 'robot.yaml')
    sim_params = os.path.join(pkg, 'config', 'sim.yaml')
    slam_params = os.path.join(get_package_share_directory('slam_ros2'), 'config', 'slam_params.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'mini_auto.rviz')

    use_rviz = LaunchConfiguration('use_rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('cam_x', default_value='0.15'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.25'),
        DeclareLaunchArgument('cam_pitch', default_value='0.35'),  # 下向きが正 [rad]

        Node(package='mini_auto', executable='sim_robot', name='sim_robot', output='screen',
             parameters=[robot_params, sim_params]),
        # LiDAR は機体中心 (sim_robot はこの前提でスキャンを作る)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),
        # D456 の取り付け (base_link -> camera_link)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_camera',
             arguments=['--x', LaunchConfiguration('cam_x'), '--y', LaunchConfiguration('cam_y'),
                        '--z', LaunchConfiguration('cam_z'), '--pitch', LaunchConfiguration('cam_pitch'),
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),
        # 実機では realsense2_camera が出す camera_link -> 光学座標 (z 前, x 右, y 下)
        Node(package='tf2_ros', executable='static_transform_publisher', name='camera_to_optical',
             arguments=['--roll', '-1.5707963', '--yaw', '-1.5707963',
                        '--frame-id', 'camera_link', '--child-frame-id', 'camera_depth_optical_frame']),

        Node(package='mini_auto', executable='omni_drive', name='omni_drive', output='screen',
             parameters=[robot_params]),
        Node(package='mini_auto', executable='tracking_odom', name='tracking_odom', output='screen',
             parameters=[robot_params]),
        Node(package='slam_ros2', executable='slam_node', name='slam_node', output='screen',
             parameters=[slam_params]),
        Node(package='mini_auto', executable='depth_obstacle', name='depth_obstacle', output='screen',
             parameters=[robot_params]),
        Node(package='mini_auto', executable='simple_nav', name='simple_nav', output='screen',
             parameters=[robot_params]),

        Node(package='rviz2', executable='rviz2', name='rviz2', arguments=['-d', rviz_config],
             condition=IfCondition(use_rviz)),
    ])
