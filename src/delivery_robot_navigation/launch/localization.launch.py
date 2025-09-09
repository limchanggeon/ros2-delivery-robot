#!/usr/bin/env python3
"""
EKF 기반 로봇 위치 추정 런치 파일
이중 EKF (local + global) 및 navsat_transform 실행

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """EKF 위치 추정 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_navigation'),
            'config',
            'ekf.yaml'
        ]),
        description='EKF 파라미터 파일 경로'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    
    # 시뮬레이션 시간 설정
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # EKF Local 노드 (odom → base_link)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_odom',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/local'),
        ]
    )
    
    # EKF Global 노드 (map → odom)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_map',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odometry/filtered', '/odometry/filtered/global'),
        ]
    )
    
    # NavSat Transform 노드 (GPS → Map 좌표 변환)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),
            ('/odometry/filtered', '/odometry/filtered/global'),
            ('/odometry/gps', '/odometry/gps'),
        ]
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_params_file,
        
        # 글로벌 설정
        set_use_sim_time,
        
        # 노드들
        ekf_local_node,
        ekf_global_node,
        navsat_transform_node,
    ])