#!/bin/bash

# ROS 2 배달 로봇 프로젝트 빌드 및 실행 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

set -e  # 오류 시 스크립트 중단

echo "========================================="
echo "ROS 2 배달 로봇 시스템 빌드 시작"
echo "========================================="

# ROS 2 환경 설정 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS 2 환경이 설정되지 않았습니다. setup.bash를 소스하세요."
    echo "   source /opt/ros/humble/setup.bash"
    exit 1
fi

echo "✅ ROS 2 $ROS_DISTRO 환경 감지됨"

# 작업공간 확인
if [ ! -f "src/CMakeLists.txt" ] && [ ! -d "src/delivery_robot_description" ]; then
    echo "❌ ROS 2 작업공간이 아닙니다. ros2_ws 디렉토리에서 실행하세요."
    exit 1
fi

# 의존성 설치
echo "📦 의존성 설치 중..."
sudo apt update

# 기본 ROS 2 패키지 설치
echo "🔧 기본 ROS 2 패키지 설치 중..."
sudo apt install -y \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-common \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gazebo-ros \
    ros-${ROS_DISTRO}-gazebo-plugins \
    ros-${ROS_DISTRO}-joint-state-publisher \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-vision-msgs \
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-diff-drive-controller \
    ros-${ROS_DISTRO}-joint-state-broadcaster

# Python 의존성 설치
echo "🐍 Python 의존성 설치 중..."
pip3 install --user \
    ultralytics \
    torch \
    torchvision \
    opencv-python \
    numpy \
    requests \
    geopy \
    pyserial \
    cryptography \
    qrcode \
    pyzbar

# YOLOv8 모델 다운로드
echo "🤖 YOLOv8 모델 다운로드 중..."
mkdir -p src/delivery_robot_perception/models
cd src/delivery_robot_perception/models
if [ ! -f "yolov8n.pt" ]; then
    echo "YOLOv8n 모델 다운로드 중..."
    wget -q https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt
    echo "✅ YOLOv8n 모델 다운로드 완료"
fi
cd ../../../

# rosdep 업데이트 및 의존성 설치
echo "📚 rosdep 의존성 설치 중..."
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 빌드 실행
echo "🔨 패키지 빌드 중..."
colcon build --symlink-install

# 빌드 결과 확인
if [ $? -eq 0 ]; then
    echo "✅ 빌드 성공!"
    echo ""
    echo "========================================="
    echo "🚀 시스템 실행 가이드"
    echo "========================================="
    echo ""
    echo "1. 환경 설정:"
    echo "   source install/setup.bash"
    echo ""
    echo "2. 전체 시스템 실행:"
    echo "   ros2 launch delivery_robot_mission full_system_launch.py"
    echo ""
    echo "3. 개별 서브시스템 실행:"
    echo "   # 로봇 모델"
    echo "   ros2 launch delivery_robot_description robot_description.launch.py"
    echo ""
    echo "   # 위치 추정 (EKF)"
    echo "   ros2 launch delivery_robot_navigation localization.launch.py"
    echo ""
    echo "   # 내비게이션"
    echo "   ros2 launch delivery_robot_navigation navigation.launch.py"
    echo ""
    echo "   # 인식 시스템"
    echo "   ros2 run delivery_robot_perception yolo_inference_node"
    echo "   ros2 run delivery_robot_perception camera_driver_node"
    echo ""
    echo "   # 보안 시스템"
    echo "   ros2 run delivery_robot_security authentication_node"
    echo ""
    echo "   # 임무 관리"
    echo "   ros2 run delivery_robot_mission mission_control_node"
    echo ""
    echo "4. 시각화 (RViz):"
    echo "   rviz2"
    echo ""
    echo "========================================="
    echo "✨ 배달 로봇 시스템 준비 완료!"
    echo "========================================="
else
    echo "❌ 빌드 실패!"
    echo "오류를 확인하고 다시 시도하세요."
    exit 1
fi