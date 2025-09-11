#!/bin/bash

# NARCHON Integrated Control System Integration Script
# This script integrates the control system with the existing ROS2 delivery robot project

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

echo "=================================================================="
echo "🤖 NARCHON Control System Integration with ROS2 Delivery Robot"
echo "=================================================================="

# 1. ROS2 환경 확인
print_status "Checking ROS2 environment..."

if [ -z "$ROS_DISTRO" ]; then
    print_warning "ROS2 environment not sourced. Attempting to source..."
    
    # 일반적인 ROS2 설치 경로들 확인
    ROS2_SETUP_PATHS=(
        "/opt/ros/humble/setup.bash"
        "/opt/ros/foxy/setup.bash" 
        "/opt/ros/galactic/setup.bash"
        "/opt/ros/iron/setup.bash"
    )
    
    for setup_path in "${ROS2_SETUP_PATHS[@]}"; do
        if [ -f "$setup_path" ]; then
            print_status "Found ROS2 setup at: $setup_path"
            source "$setup_path"
            break
        fi
    done
    
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 not found. Please install ROS2 first."
        echo "Install guide: https://docs.ros.org/en/humble/Installation.html"
        exit 1
    fi
fi

print_success "ROS2 $ROS_DISTRO detected"

# 2. 작업공간 설정
print_status "Setting up workspace..."

if [ ! -f "src/delivery_robot_perception/package.xml" ]; then
    print_error "ROS2 delivery robot packages not found!"
    echo "Please ensure you're in the ros2-delivery-robot workspace"
    exit 1
fi

print_success "ROS2 delivery robot packages found"

# 3. 통합 관제 시스템 노드들을 ROS2 패키지로 복사
print_status "Integrating control system nodes..."

# perception 패키지에 웹 브리지 노드 추가
if [ -f "Integrated Control System/web_bridge_node.py" ]; then
    cp "Integrated Control System/web_bridge_node.py" "src/delivery_robot_perception/delivery_robot_perception/"
    print_success "Web bridge node integrated"
fi

if [ -f "Integrated Control System/status_publisher_node.py" ]; then
    cp "Integrated Control System/status_publisher_node.py" "src/delivery_robot_perception/delivery_robot_perception/"
    print_success "Status publisher node integrated"
fi

# 4. launch 파일 생성
print_status "Creating integrated launch files..."

mkdir -p "Integrated Control System/launch"

cat > "Integrated Control System/launch/integrated_system.launch.py" << 'EOF'
"""
Integrated launch file for NARCHON Control System + ROS2 Delivery Robot
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    IncludeLaunchDescription,
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Launch arguments
    robot_namespace = LaunchConfiguration('robot_namespace')
    backend_host = LaunchConfiguration('backend_host')
    backend_port = LaunchConfiguration('backend_port')
    
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace',
        default_value='robot_01',
        description='Namespace for the robot'
    )
    
    declare_backend_host = DeclareLaunchArgument(
        'backend_host', 
        default_value='localhost',
        description='Backend server host'
    )
    
    declare_backend_port = DeclareLaunchArgument(
        'backend_port',
        default_value='8000', 
        description='Backend server port'
    )
    
    # Robot system launch
    robot_system_launch = GroupAction([
        # Robot description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('delivery_robot_description'),
                    'launch',
                    'robot_description.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': robot_namespace,
            }.items()
        ),
        
        # Navigation system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('delivery_robot_navigation'),
                    'launch', 
                    'navigation.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': robot_namespace,
            }.items()
        ),
        
        # Mission system
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('delivery_robot_mission'),
                    'launch',
                    'mission.launch.py'
                ])
            ]),
            launch_arguments={
                'namespace': robot_namespace,
            }.items()
        ),
    ])
    
    # Control system nodes
    control_system_nodes = GroupAction([
        # Status publisher node
        Node(
            package='delivery_robot_perception',
            executable='status_publisher_node',
            name='status_publisher',
            namespace=robot_namespace,
            parameters=[{
                'publish_rate': 2.0,
                'include_system_stats': True,
                'include_wifi_info': True,
            }],
            output='screen'
        ),
        
        # Web bridge node  
        Node(
            package='delivery_robot_perception',
            executable='web_bridge_node',
            name='web_bridge',
            namespace=robot_namespace,
            parameters=[{
                'backend_host': backend_host,
                'backend_port': backend_port,
                'robot_id': robot_namespace,
                'reconnect_interval': 5.0,
            }],
            output='screen'
        ),
    ])
    
    # Backend server (optional - can be run separately)
    backend_server = ExecuteProcess(
        cmd=[
            'python3', '-m', 'uvicorn', 
            'FastAPI:app',
            '--host', '0.0.0.0',
            '--port', backend_port,
            '--reload'
        ],
        cwd='Integrated Control System',
        output='screen',
        prefix='gnome-terminal -- '  # Run in separate terminal
    )
    
    return LaunchDescription([
        declare_robot_namespace,
        declare_backend_host, 
        declare_backend_port,
        robot_system_launch,
        control_system_nodes,
        # backend_server,  # Uncomment to auto-start backend
    ])
EOF

print_success "Integrated launch file created"

# 5. 설정 파일 업데이트
print_status "Updating configuration files..."

# perception 패키지 setup.py 업데이트
if [ -f "src/delivery_robot_perception/setup.py" ]; then
    # 백업 생성
    cp "src/delivery_robot_perception/setup.py" "src/delivery_robot_perception/setup.py.backup"
    
    # entry_points에 새 노드들 추가
    python3 << 'EOF'
import re

with open('src/delivery_robot_perception/setup.py', 'r') as f:
    content = f.read()

# entry_points 섹션 찾기
entry_points_pattern = r"entry_points\s*=\s*\{[^}]*'console_scripts':\s*\[(.*?)\]"
match = re.search(entry_points_pattern, content, re.DOTALL)

if match:
    scripts = match.group(1)
    
    # 새 노드들이 없다면 추가
    new_scripts = [
        "'status_publisher_node = delivery_robot_perception.status_publisher_node:main'",
        "'web_bridge_node = delivery_robot_perception.web_bridge_node:main'"
    ]
    
    for script in new_scripts:
        if script not in scripts:
            # 마지막 스크립트 뒤에 추가
            scripts = scripts.rstrip().rstrip(',') + ',\n        ' + script
    
    # 원본 내용 교체
    new_entry_points = f"entry_points={{\n        'console_scripts': [\n{scripts}\n        ],\n    }}"
    content = re.sub(entry_points_pattern, new_entry_points, content, flags=re.DOTALL)
    
    with open('src/delivery_robot_perception/setup.py', 'w') as f:
        f.write(content)
    
    print("✅ setup.py updated with new console scripts")
else:
    print("❌ Could not find entry_points section in setup.py")
EOF
fi

# 6. 빌드 및 설치
print_status "Building integrated system..."

# ROS2 워크스페이스 빌드
if command -v colcon &> /dev/null; then
    print_status "Building with colcon..."
    colcon build --packages-select delivery_robot_perception delivery_robot_mission
    
    if [ $? -eq 0 ]; then
        print_success "ROS2 packages built successfully"
        
        # 환경 설정
        source install/setup.bash
        print_success "Workspace sourced"
    else
        print_error "Build failed"
        exit 1
    fi
else
    print_warning "colcon not found. Please install: sudo apt install python3-colcon-common-extensions"
fi

# 7. Python 의존성 설치
print_status "Installing Python dependencies for control system..."
cd "Integrated Control System"

pip3 install --user \
    fastapi[all] \
    uvicorn[standard] \
    websockets \
    psutil \
    python-multipart \
    aiofiles \
    python-dotenv

print_success "Python dependencies installed"

# 8. 데이터베이스 초기화
print_status "Initializing database..."
python3 -c "
import sqlite3
import json
from datetime import datetime

conn = sqlite3.connect('robot_control.db')
cursor = conn.cursor()

# 테이블 생성
tables = [
    '''CREATE TABLE IF NOT EXISTS robots (
        robot_id TEXT PRIMARY KEY,
        name TEXT,
        model TEXT,
        last_seen TIMESTAMP,
        current_status TEXT
    )''',
    '''CREATE TABLE IF NOT EXISTS telemetry (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        robot_id TEXT,
        timestamp TIMESTAMP,
        battery_level REAL,
        cpu_usage REAL,
        memory_usage REAL,
        position_x REAL,
        position_y REAL,
        velocity_linear REAL,
        velocity_angular REAL,
        wifi_signal INTEGER,
        FOREIGN KEY (robot_id) REFERENCES robots (robot_id)
    )''',
    '''CREATE TABLE IF NOT EXISTS missions (
        mission_id TEXT PRIMARY KEY,
        name TEXT,
        description TEXT,
        definition TEXT,
        created_at TIMESTAMP
    )''',
    '''CREATE TABLE IF NOT EXISTS robot_logs (
        id INTEGER PRIMARY KEY AUTOINCREMENT,
        robot_id TEXT,
        timestamp TIMESTAMP,
        log_level TEXT,
        message TEXT,
        FOREIGN KEY (robot_id) REFERENCES robots (robot_id)
    )'''
]

for table_sql in tables:
    cursor.execute(table_sql)

conn.commit()
conn.close()
print('Database initialized')
"

print_success "Database initialized"

cd ..

echo ""
echo "=================================================================="
echo "🎉 Integration Complete!"
echo "=================================================================="
echo ""
echo "📍 System Components:"
echo "   ├── ROS2 Delivery Robot (src/delivery_robot_*)"
echo "   ├── NARCHON Control System (Integrated Control System/)"
echo "   ├── Enhanced Perception Nodes (integrated)"
echo "   └── Integrated Launch Files (Integrated Control System/launch/)"
echo ""
echo "🚀 Quick Start Commands:"
echo ""
echo "1. Start Control System Backend:"
echo "   cd 'Integrated Control System'"
echo "   python3 lanch.py"
echo ""
echo "2. Launch Robot + Control Integration:"
echo "   ros2 launch Integrated\ Control\ System/launch/integrated_system.launch.py"
echo ""
echo "3. Manual Node Launch:"
echo "   ros2 run delivery_robot_perception status_publisher_node --ros-args -r __ns:=/robot_01"
echo "   ros2 run delivery_robot_perception web_bridge_node --ros-args -r __ns:=/robot_01"
echo ""
echo "📊 Access Points:"
echo "   🌍 Web Dashboard: http://localhost:8000"
echo "   📖 API Docs: http://localhost:8000/docs"
echo "   🔌 WebSocket: ws://localhost:8000/ws/ui"
echo ""
echo "=================================================================="

print_success "NARCHON Integrated Control System successfully integrated with ROS2 Delivery Robot!"