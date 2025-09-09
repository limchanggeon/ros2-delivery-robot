#!/usr/bin/env python3
"""
Nav2 내비게이션 스택 런치 파일
지도 서버, AMCL, 경로 계획기, 제어기 등 실행

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """Nav2 내비게이션 런치 파일"""
    
    # 런치 인수 선언
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='시뮬레이션 시간 사용 여부'
    )
    
    declare_map_yaml_file = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_navigation'),
            'maps',
            'warehouse.yaml'
        ]),
        description='로드할 지도 파일 경로'
    )
    
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_navigation'),
            'config',
            'nav2_params.yaml'
        ]),
        description='Nav2 파라미터 파일 경로'
    )
    
    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Nav2 노드들 자동 시작 여부'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    
    # 시뮬레이션 시간 설정
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # Nav2 bringup 실행
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('nav2_bringup'),
            '/launch/bringup_launch.py'
        ]),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': autostart,
        }.items()
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_map_yaml_file,
        declare_params_file,
        declare_autostart,
        
        # 글로벌 설정
        set_use_sim_time,
        
        # Nav2 스택
        nav2_bringup,
    ])