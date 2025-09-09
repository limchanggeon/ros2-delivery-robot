#!/usr/bin/env python3
"""
배달 로봇 전체 시스템 런치 파일
모든 서브시스템을 통합하여 실행하는 메인 런치 파일

작성자: 배달로봇팀  
날짜: 2025-09-09
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter


def generate_launch_description():
    """전체 배달 로봇 시스템을 실행하는 런치 파일"""
    
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
    
    declare_ekf_params_file = DeclareLaunchArgument(
        'ekf_params_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('delivery_robot_navigation'),
            'config', 
            'ekf.yaml'
        ]),
        description='EKF 파라미터 파일 경로'
    )
    
    declare_rviz_config = DeclareLaunchArgument(
        'rviz_config',
        default_value='',
        description='RViz 설정 파일 경로 (빈 값이면 기본 설정 사용)'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='RViz 실행 여부'
    )
    
    declare_enable_perception = DeclareLaunchArgument(
        'enable_perception',
        default_value='true',
        description='인식 시스템 활성화 여부'
    )
    
    declare_enable_security = DeclareLaunchArgument(
        'enable_security',
        default_value='true',
        description='보안 시스템 활성화 여부'
    )
    
    # 런치 설정 변수
    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    ekf_params_file = LaunchConfiguration('ekf_params_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_rviz = LaunchConfiguration('use_rviz')
    enable_perception = LaunchConfiguration('enable_perception')
    enable_security = LaunchConfiguration('enable_security')
    
    # 시뮬레이션 시간 설정
    set_use_sim_time = SetParameter(name='use_sim_time', value=use_sim_time)
    
    # 1. 로봇 모델 및 상태 발행자
    robot_description_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_description'),
                '/launch/robot_description.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ])
    
    # 2. 센서 융합 및 위치 추정 (robot_localization)
    localization_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_navigation'),
                '/launch/localization.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'params_file': ekf_params_file,
            }.items()
        )
    ])
    
    # 3. 내비게이션 스택 (Nav2)
    navigation_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_navigation'),
                '/launch/navigation.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml_file,
                'params_file': params_file,
            }.items()
        )
    ])
    
    # 4. 인식 시스템 (YOLOv8, 카메라)
    perception_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_perception'),
                '/launch/perception.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ], condition=IfCondition(enable_perception))
    
    # 5. 보안 시스템 (QR 코드, 도어 제어)
    security_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_security'),
                '/launch/security.launch.py'  
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ], condition=IfCondition(enable_security))
    
    # 6. 임무 관리 및 제어
    mission_control_group = GroupAction([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('delivery_robot_mission'),
                '/launch/mission.launch.py'
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
            }.items()
        )
    ])
    
    # 7. 하드웨어 인터페이스 (ros2_control) - 젯슨 환경에서 선택적 실행
    # control_group = GroupAction([
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource([
    #             FindPackageShare('delivery_robot_control'),
    #             '/launch/control.launch.py'
    #         ]),
    #         launch_arguments={
    #             'use_sim_time': use_sim_time,
    #         }.items()
    #     )
    # ])
    
    # 8. RViz 시각화 (젯슨에서는 선택적, 헤드리스 모드 지원)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz)
    )
    
    # 9. 시스템 모니터링 노드 (선택사항)
    system_monitor_node = Node(
        package='delivery_robot_mission',
        executable='system_monitor_node',
        name='system_monitor',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # 런치 인수
        declare_use_sim_time,
        declare_map_yaml_file,
        declare_params_file,
        declare_ekf_params_file,
        declare_rviz_config,
        declare_use_rviz,
        declare_enable_perception,
        declare_enable_security,
        
        # 글로벌 설정
        set_use_sim_time,
        
        # 시스템 구성 요소들 (순서 중요)
        robot_description_group,      # 1. 로봇 모델
        # control_group,               # 2. 하드웨어 제어 (젯슨에서 선택적)
        localization_group,          # 3. 위치 추정
        navigation_group,            # 4. 내비게이션
        perception_group,            # 5. 인식
        security_group,              # 6. 보안
        mission_control_group,       # 7. 임무 관리
        
        # 시각화 및 모니터링
        rviz_node,                   # 8. RViz
        system_monitor_node,         # 9. 시스템 모니터
    ])


if __name__ == '__main__':
    generate_launch_description()