#!/usr/bin/env python3
"""
보안 시스템 런치 파일
QR 코드 인증 및 도어 제어 노드 실행

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """보안 시스템 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_security'),
            'config',
            'qr_params.yaml'
        ]),
        description='보안 설정 파일 경로'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # QR 코드 인증 노드
    authentication_node = Node(
        package='delivery_robot_security',
        executable='authentication_node',
        name='authentication',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_config_file,
        
        # 노드들
        authentication_node,
    ])