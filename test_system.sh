#!/bin/bash

# ROS 2 배달 로봇 시스템 테스트 스크립트
# 각 컴포넌트의 기본 기능을 테스트합니다.

echo "========================================="
echo "🧪 ROS 2 배달 로봇 시스템 테스트"
echo "========================================="

# ROS 2 환경 확인
source install/setup.bash

echo "1️⃣ 패키지 존재 확인..."
PACKAGES=("delivery_robot_description" "delivery_robot_navigation" "delivery_robot_perception" "delivery_robot_security" "delivery_robot_control" "delivery_robot_mission")

for pkg in "${PACKAGES[@]}"; do
    if ros2 pkg list | grep -q "$pkg"; then
        echo "✅ $pkg 패키지 확인"
    else
        echo "❌ $pkg 패키지 없음"
    fi
done

echo ""
echo "2️⃣ 런치 파일 확인..."
LAUNCH_FILES=(
    "delivery_robot_description robot_description.launch.py"
    "delivery_robot_navigation localization.launch.py"
    "delivery_robot_navigation navigation.launch.py"
    "delivery_robot_perception perception.launch.py"
    "delivery_robot_security security.launch.py"
    "delivery_robot_mission mission.launch.py"
    "delivery_robot_mission full_system_launch.py"
)

for launch in "${LAUNCH_FILES[@]}"; do
    if ros2 launch $launch --show-args &>/dev/null; then
        echo "✅ $launch 런치 파일 확인"
    else
        echo "❌ $launch 런치 파일 문제"
    fi
done

echo ""
echo "3️⃣ 노드 실행 파일 확인..."
EXECUTABLES=(
    "delivery_robot_perception yolo_inference_node"
    "delivery_robot_perception camera_driver_node"
    "delivery_robot_security authentication_node"
    "delivery_robot_mission mission_control_node"
    "delivery_robot_mission system_monitor_node"
)

for exec in "${EXECUTABLES[@]}"; do
    if ros2 run $exec --help &>/dev/null; then
        echo "✅ $exec 실행 파일 확인"
    else
        echo "❌ $exec 실행 파일 문제"
    fi
done

echo ""
echo "4️⃣ 의존성 확인..."
echo "📦 Python 패키지 확인..."
PYTHON_DEPS=("ultralytics" "torch" "opencv-python" "numpy" "requests")

for dep in "${PYTHON_DEPS[@]}"; do
    if python3 -c "import ${dep//-/_}" &>/dev/null; then
        echo "✅ $dep 설치됨"
    else
        echo "❌ $dep 없음 - pip3 install $dep"
    fi
done

echo ""
echo "🔧 ROS 2 패키지 확인..."
ROS_DEPS=("nav2_bringup" "robot_localization" "cv_bridge" "vision_msgs")

for dep in "${ROS_DEPS[@]}"; do
    if ros2 pkg list | grep -q "$dep"; then
        echo "✅ $dep 설치됨"
    else
        echo "❌ $dep 없음 - sudo apt install ros-$ROS_DISTRO-${dep//_/-}"
    fi
done

echo ""
echo "5️⃣ 설정 파일 확인..."
CONFIG_FILES=(
    "src/delivery_robot_navigation/config/nav2_params.yaml"
    "src/delivery_robot_navigation/config/ekf.yaml"
    "src/delivery_robot_perception/config/yolo_config.yaml"
    "src/delivery_robot_security/config/qr_params.yaml"
    "src/delivery_robot_mission/config/mission_params.yaml"
    "src/delivery_robot_control/config/ros2_control.yaml"
)

for config in "${CONFIG_FILES[@]}"; do
    if [[ -f "$config" ]]; then
        echo "✅ $config 존재"
    else
        echo "❌ $config 없음"
    fi
done

echo ""
echo "6️⃣ 모델 파일 확인..."
MODELS_DIR="src/delivery_robot_perception/models"
if [[ -d "$MODELS_DIR" ]]; then
    if [[ -f "$MODELS_DIR/yolov8n.pt" ]]; then
        echo "✅ YOLOv8 모델 파일 존재"
    else
        echo "❌ YOLOv8 모델 파일 없음 - ./build_and_run.sh 실행"
    fi
else
    echo "❌ 모델 디렉토리 없음"
fi

echo ""
echo "========================================="
echo "🎯 빠른 테스트 명령어"
echo "========================================="
echo ""
echo "# 로봇 모델 테스트"
echo "ros2 launch delivery_robot_description robot_description.launch.py use_gui:=true"
echo ""
echo "# 카메라 테스트 (카메라 연결 필요)"
echo "ros2 run delivery_robot_perception camera_driver_node"
echo ""
echo "# YOLOv8 테스트 (카메라 및 모델 필요)"
echo "ros2 run delivery_robot_perception yolo_inference_node"
echo ""
echo "# 시스템 모니터 테스트"
echo "ros2 run delivery_robot_mission system_monitor_node"
echo ""
echo "# 전체 시스템 테스트 (시뮬레이션)"
echo "ros2 launch delivery_robot_mission full_system_launch.py use_sim_time:=true"
echo ""
echo "========================================="
echo "✨ 테스트 완료!"
echo "========================================="