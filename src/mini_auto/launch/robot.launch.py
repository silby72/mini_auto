"""実機用: LiDAR(YDLIDAR TG30) + IMU(WT901C) + 足回り + 3輪オドメトリ + slam_ros2 + 簡易ナビ。

ros2can (GUI) は別ターミナルで起動し、計測輪エンコーダの原点セットまで済ませておくこと。
  ros2 launch ros2can ros2can.launch.py
LiDAR ドライバを自前で起動する場合は launch_lidar:=false。
LiDAR の取り付け位置は laser_x / laser_y / laser_yaw で与える。
IMU は wit_node (wit_node_ros2) で /imu を出す。launch_imu:=false で自前起動。
D456 は realsense2_camera で深度だけ 848x480 15fps で出し、depth_obstacle で障害物スキャンにする。
取り付けは cam_x / cam_y / cam_z / cam_pitch (下向きが正)。launch_camera:=false で自前起動。
WT901C 側は出力レート 100 Hz・6軸モード(磁気を使わない)に設定しておくこと。
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
    slam_pkg = get_package_share_directory('slam_ros2')
    robot_params = os.path.join(pkg, 'config', 'robot.yaml')
    slam_params = os.path.join(slam_pkg, 'config', 'slam_params.yaml')
    lidar_params = os.path.join(slam_pkg, 'config', 'tg30.yaml')
    rviz_config = os.path.join(pkg, 'rviz', 'mini_auto.rviz')

    use_rviz = LaunchConfiguration('use_rviz')
    launch_lidar = LaunchConfiguration('launch_lidar')
    launch_imu = LaunchConfiguration('launch_imu')
    launch_camera = LaunchConfiguration('launch_camera')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('launch_lidar', default_value='true'),
        DeclareLaunchArgument('launch_imu', default_value='true'),
        DeclareLaunchArgument('imu_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('imu_baud_rate', default_value='115200'),
        DeclareLaunchArgument('launch_camera', default_value='true'),
        DeclareLaunchArgument('cam_x', default_value='0.15'),
        DeclareLaunchArgument('cam_y', default_value='0.0'),
        DeclareLaunchArgument('cam_z', default_value='0.25'),
        DeclareLaunchArgument('cam_pitch', default_value='0.35'),  # 下向きが正 [rad]
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),

        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen', parameters=[lidar_params],
             condition=IfCondition(launch_lidar)),
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_laser',
             arguments=['--x', LaunchConfiguration('laser_x'),
                        '--y', LaunchConfiguration('laser_y'),
                        '--yaw', LaunchConfiguration('laser_yaw'),
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),

        Node(package='wit_node', executable='wit_node', name='wit_node', output='screen',
             parameters=[{'port': LaunchConfiguration('imu_port'),
                          'baud_rate': LaunchConfiguration('imu_baud_rate'),
                          'frame_id': 'imu_link',
                          'publish_hz': 100.0}],
             condition=IfCondition(launch_imu)),

        # D456: 障害物検知用に深度だけ。RGB/IR/点群/IMU は切って USB と CPU を節約
        Node(package='realsense2_camera', executable='realsense2_camera_node',
             namespace='camera', name='camera', output='screen',
             parameters=[{'enable_depth': True,
                          'depth_module.depth_profile': '848x480x15',
                          'enable_color': False,
                          'enable_infra1': False,
                          'enable_infra2': False,
                          'enable_gyro': False,
                          'enable_accel': False,
                          'pointcloud.enable': False}],
             condition=IfCondition(launch_camera)),
        # D456 の取り付け (base_link -> camera_link)
        Node(package='tf2_ros', executable='static_transform_publisher', name='base_to_camera',
             arguments=['--x', LaunchConfiguration('cam_x'), '--y', LaunchConfiguration('cam_y'),
                        '--z', LaunchConfiguration('cam_z'), '--pitch', LaunchConfiguration('cam_pitch'),
                        '--frame-id', 'base_link', '--child-frame-id', 'camera_link']),

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
