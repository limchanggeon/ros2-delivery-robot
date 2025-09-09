#!/usr/bin/env python3
"""
배달 로봇 하드웨어 제어 런치 파일
ros2_control을 사용한 하드웨어 인터페이스 실행

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
    """하드웨어 제어 시스템을 실행하는 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_control'),
            'config',
            'ros2_control.yaml'
        ]),
        description='ros2_control 설정 파일 경로'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    config_file = LaunchConfiguration('config_file')
    
    # Controller Manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        name='controller_manager',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # Differential Drive Controller
    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen'
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen'
    )
    
    # GPIO Controller (로봇 특화)
    gpio_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gpio_controller'],
        output='screen'
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_config_file,
        
        # 컨트롤러들
        controller_manager,
        diff_drive_spawner,
        joint_state_broadcaster_spawner,
        gpio_controller_spawner,
    ])


if __name__ == '__main__':
    generate_launch_description()