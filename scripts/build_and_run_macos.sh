#!/bin/bash

# macOS용 ROS 2 배달 로봇 프로젝트 빌드 스크립트
# Docker를 사용한 Linux 환경에서 빌드

set -e

echo "========================================="
echo "macOS용 ROS 2 배달 로봇 시스템 빌드"
echo "========================================="

# Docker 설치 확인
if ! command -v docker &> /dev/null; then
    echo "❌ Docker가 설치되지 않았습니다."
    echo "Docker Desktop을 설치하세요: https://docs.docker.com/desktop/install/mac-install/"
    exit 1
fi

# Docker 실행 확인
if ! docker info &> /dev/null; then
    echo "❌ Docker가 실행되고 있지 않습니다."
    echo "Docker Desktop을 시작하세요."
    exit 1
fi

echo "✅ Docker 환경 확인됨"

# ROS 2 Humble Docker 이미지 다운로드
echo "📦 ROS 2 Humble Docker 이미지 다운로드 중..."
docker pull osrf/ros:humble-desktop

# 현재 디렉토리를 Docker 컨테이너에 마운트하여 빌드
echo "🔨 Docker 컨테이너에서 빌드 중..."
docker run --rm -it \
    -v $(pwd):/workspace \
    -w /workspace \
    osrf/ros:humble-desktop \
    bash -c "
        set -e
        echo '✅ ROS 2 Humble 환경 준비됨'
        
        # 의존성 업데이트
        apt update
        
        # 기본 빌드 도구 설치
        apt install -y python3-pip python3-colcon-common-extensions
        
        # Python 의존성 설치
        pip3 install ultralytics torch torchvision opencv-python numpy requests geopy pyserial cryptography qrcode pyzbar
        
        # rosdep 초기화 및 의존성 설치
        rosdep init || true
        rosdep update
        rosdep install --from-paths src --ignore-src -r -y
        
        # 빌드 실행
        source /opt/ros/humble/setup.bash
        colcon build --symlink-install
        
        echo '✅ 빌드 완료!'
    "

echo ""
echo "========================================="
echo "🚀 Docker 환경에서 실행 가이드"
echo "========================================="
echo ""
echo "1. Docker 컨테이너에서 ROS 2 환경 시작:"
echo "docker run --rm -it \\"
echo "    -v \$(pwd):/workspace \\"
echo "    -w /workspace \\"
echo "    osrf/ros:humble-desktop \\"
echo "    bash"
echo ""
echo "2. 컨테이너 내에서 환경 설정:"
echo "source /opt/ros/humble/setup.bash"
echo "source install/setup.bash"
echo ""
echo "3. 시스템 실행:"
echo "ros2 launch delivery_robot_mission full_system_launch.py"
echo ""
echo "========================================="
echo "✨ macOS용 빌드 완료!"
echo "========================================="