#!/usr/bin/env python3
"""
임무 관리 런치 파일
미션 컨트롤 및 API 인터페이스 노드 실행

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
    """임무 관리 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_mission'),
            'config',
            'mission_params.yaml'
        ]),
        description='임무 설정 파일 경로'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # 임무 제어 노드
    mission_control_node = Node(
        package='delivery_robot_mission',
        executable='mission_control_node',
        name='mission_control',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # 시스템 모니터 노드
    system_monitor_node = Node(
        package='delivery_robot_mission',
        executable='system_monitor_node',
        name='system_monitor',
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
        mission_control_node,
        system_monitor_node,
    ])