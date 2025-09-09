#!/bin/bash

# 젯슨 오린 나노 Ubuntu 20.04 / ROS 2 Foxy 빌드 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

set -e

echo "========================================="
echo "젯슨 오린 나노 ROS 2 Foxy 빌드 시작"
echo "========================================="

# ROS 2 환경 설정 확인
if [ -z "$ROS_DISTRO" ]; then
    echo "🔧 ROS 2 환경 설정 중..."
    source /opt/ros/foxy/setup.bash
    export ROS_DISTRO=foxy
fi

echo "✅ ROS 2 $ROS_DISTRO 환경 감지됨"

# 작업공간 확인
if [ ! -d "src/delivery_robot_description" ]; then
    echo "❌ ROS 2 작업공간이 아닙니다. 프로젝트 루트 디렉토리에서 실행하세요."
    exit 1
fi

# 젯슨 환경 최적화 설정
echo "⚡ 젯슨 환경 최적화 설정..."
export JETSON_MODEL=orin-nano
export CUDA_ARCH_BIN="8.7"
export NVIDIA_TF32_OVERRIDE=0

# 시스템 의존성 업데이트
echo "📦 시스템 패키지 업데이트 중..."
sudo apt update

# 기본 ROS 2 Foxy 패키지 설치
echo "🔧 ROS 2 Foxy 패키지 설치 중..."
sudo apt install -y \
    ros-foxy-navigation2 \
    ros-foxy-nav2-bringup \
    ros-foxy-robot-localization \
    ros-foxy-slam-toolbox \
    ros-foxy-gazebo-ros \
    ros-foxy-gazebo-plugins \
    ros-foxy-joint-state-publisher \
    ros-foxy-joint-state-publisher-gui \
    ros-foxy-robot-state-publisher \
    ros-foxy-xacro \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep

# Python 의존성 설치 (젯슨 최적화)
echo "🐍 Python 의존성 설치 중 (젯슨 최적화)..."
chmod +x install_python_deps.sh
./install_python_deps.sh

# 젯슨 전용 PyTorch CUDA 설치 확인
echo "🤖 젯슨용 PyTorch CUDA 설치 확인 중..."
if python3 -c "import torch; print(f'PyTorch {torch.__version__} - CUDA available: {torch.cuda.is_available()}')" 2>/dev/null; then
    echo "✅ PyTorch 이미 설치됨"
else
    echo "❌ PyTorch가 설치되지 않았거나 CUDA를 지원하지 않습니다."
    echo "install_python_deps.sh 스크립트가 젯슨용 CUDA PyTorch를 자동으로 설치했어야 합니다."
    echo "수동 설치가 필요한 경우:"
    echo "wget https://developer.download.nvidia.com/compute/redist/jp/v50/pytorch/torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
    echo "pip3 install torch-2.0.0+nv23.05-cp38-cp38-linux_aarch64.whl"
fi

# rosdep 초기화 및 의존성 설치
echo "📚 rosdep 의존성 설치 중..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 젯슨 메모리 최적화 빌드
echo "🔨 메모리 최적화 빌드 실행 중..."
export MAKEFLAGS="-j2"  # 젯슨 나노는 메모리 제한으로 병렬 빌드 제한
colcon build --symlink-install \
    --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
    --parallel-workers 2

# 빌드 결과 확인
if [ $? -eq 0 ]; then
    echo "✅ 빌드 성공!"
    echo ""
    echo "========================================="
    echo "🚀 젯슨 오린 나노 실행 가이드"
    echo "========================================="
    echo ""
    echo "1. 환경 설정:"
    echo "   source /opt/ros/foxy/setup.bash"
    echo "   source install/setup.bash"
    echo ""
    echo "2. 개별 노드 실행 (테스트):"
    echo "   # YOLOv8 인식 노드"
    echo "   ros2 run delivery_robot_perception yolo_inference_node"
    echo ""
    echo "   # 카메라 드라이버"
    echo "   ros2 run delivery_robot_perception camera_driver_node"
    echo ""
    echo "   # 임무 제어"
    echo "   ros2 run delivery_robot_mission mission_control_node"
    echo ""
    echo "   # QR 인증"
    echo "   ros2 run delivery_robot_security authentication_node"
    echo ""
    echo "3. 전체 시스템 실행:"
    echo "   ros2 launch delivery_robot_mission full_system_launch.py"
    echo ""
    echo "4. 젯슨 성능 모니터링:"
    echo "   sudo jetson_clocks  # 최대 성능 모드"
    echo "   jtop                # 리소스 모니터링"
    echo ""
    echo "========================================="
    echo "✨ 젯슨 오린 나노 시스템 준비 완료!"
    echo "========================================="
else
    echo "❌ 빌드 실패!"
    echo "젯슨 환경에서 다음을 확인하세요:"
    echo "1. 메모리 사용량 (free -h)"
    echo "2. ROS 2 Foxy 설치 상태"
    echo "3. Python 의존성 설치 상태"
    exit 1
fi