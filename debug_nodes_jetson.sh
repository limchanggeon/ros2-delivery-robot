#!/bin/bash

# 젯슨 노드 실행 파일 디버깅 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

echo "========================================="
echo "젯슨 ROS 2 노드 실행 파일 디버깅"
echo "========================================="

# ROS 2 환경 설정
if [ -z "$ROS_DISTRO" ]; then
    echo "🔧 ROS 2 환경 설정 중..."
    source /opt/ros/foxy/setup.bash
fi

# 워크스페이스 환경 설정
if [ -f "install/setup.bash" ]; then
    echo "📦 워크스페이스 환경 설정 중..."
    source install/setup.bash
else
    echo "❌ install/setup.bash 없음 - 먼저 빌드가 필요합니다"
    echo "해결: ./build_and_run_jetson.sh"
    exit 1
fi

echo ""
echo "1️⃣ 패키지 확인..."
packages=(
    "delivery_robot_description"
    "delivery_robot_navigation" 
    "delivery_robot_perception"
    "delivery_robot_security"
    "delivery_robot_mission"
    "delivery_robot_control"
)

for pkg in "${packages[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "✅ $pkg 패키지 확인"
    else
        echo "❌ $pkg 패키지 문제"
    fi
done

echo ""
echo "2️⃣ 런치 파일 확인..."
launch_files=(
    "delivery_robot_description robot_description.launch.py"
    "delivery_robot_navigation localization.launch.py"
    "delivery_robot_navigation navigation.launch.py"
    "delivery_robot_perception perception.launch.py"
    "delivery_robot_security security.launch.py"
    "delivery_robot_mission mission.launch.py"
    "delivery_robot_mission full_system_launch.py"
)

for launch_info in "${launch_files[@]}"; do
    pkg=$(echo $launch_info | cut -d' ' -f1)
    launch=$(echo $launch_info | cut -d' ' -f2)
    
    if ros2 pkg list | grep -q "^$pkg$"; then
        launch_path=$(ros2 pkg prefix $pkg)/share/$pkg/launch/$launch
        if [ -f "$launch_path" ]; then
            echo "✅ $pkg $launch 런치 파일 확인"
        else
            echo "❌ $pkg $launch 런치 파일 문제"
            echo "   경로: $launch_path"
        fi
    else
        echo "❌ $pkg 패키지 없음으로 $launch 확인 불가"
    fi
done

echo ""
echo "3️⃣ 노드 실행 파일 확인..."
nodes=(
    "delivery_robot_perception yolo_inference_node"
    "delivery_robot_perception camera_driver_node"
    "delivery_robot_security authentication_node"
    "delivery_robot_mission mission_control_node"
    "delivery_robot_mission system_monitor_node"
)

for node_info in "${nodes[@]}"; do
    pkg=$(echo $node_info | cut -d' ' -f1)
    node=$(echo $node_info | cut -d' ' -f2)
    
    if ros2 pkg executables $pkg 2>/dev/null | grep -q "^$node$"; then
        echo "✅ $pkg $node 실행 파일 확인"
    else
        echo "❌ $pkg $node 실행 파일 문제"
        echo "   사용 가능한 실행 파일:"
        ros2 pkg executables $pkg 2>/dev/null | sed 's/^/     /' || echo "     (없음)"
    fi
done

echo ""
echo "4️⃣ 의존성 확인..."
echo "📦 Python 패키지 확인..."
python3 -c "
packages = ['ultralytics', 'torch', 'opencv-python', 'numpy', 'requests']
for pkg in packages:
    try:
        if pkg == 'opencv-python':
            import cv2
            print(f'✅ {pkg} 설치됨')
        else:
            __import__(pkg.replace('-', '_'))
            print(f'✅ {pkg} 설치됨')
    except ImportError:
        print(f'❌ {pkg} 없음 - pip3 install {pkg}')
"

echo ""
echo "🔧 ROS 2 패키지 확인..."
ros2_packages=("nav2_bringup" "robot_localization" "cv_bridge" "vision_msgs")
for pkg in "${ros2_packages[@]}"; do
    if ros2 pkg list | grep -q "^$pkg$"; then
        echo "✅ $pkg 설치됨"
    else
        echo "❌ $pkg 없음 - sudo apt install ros-foxy-$(echo $pkg | tr '_' '-')"
    fi
done

echo ""
echo "5️⃣ 설정 파일 확인..."
config_files=(
    "src/delivery_robot_navigation/config/nav2_params.yaml"
    "src/delivery_robot_navigation/config/ekf.yaml"
    "src/delivery_robot_perception/config/yolo_config.yaml"
    "src/delivery_robot_security/config/qr_params.yaml"
    "src/delivery_robot_mission/config/mission_params.yaml"
    "src/delivery_robot_control/config/ros2_control.yaml"
)

for file in "${config_files[@]}"; do
    if [ -f "$file" ]; then
        echo "✅ $(basename "$file") 존재"
    else
        echo "❌ $(basename "$file") 누락: $file"
    fi
done

echo ""
echo "6️⃣ 모델 파일 확인..."
if [ -d "models" ]; then
    if [ -f "models/yolov8_best.pt" ]; then
        size=$(ls -lh models/yolov8_best.pt | awk '{print $5}')
        echo "✅ YOLOv8 모델: models/yolov8_best.pt ($size)"
    else
        echo "❌ YOLOv8 모델 파일 없음: models/yolov8_best.pt"
        echo "   해결: Git LFS로 다운로드하거나 모델 파일을 복사하세요"
    fi
else
    echo "❌ 모델 디렉토리 없음"
    echo "   해결: mkdir models && 모델 파일을 models/ 디렉토리에 복사"
fi

echo ""
echo "========================================="
echo "🎯 빠른 테스트 명령어"
echo "========================================="
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