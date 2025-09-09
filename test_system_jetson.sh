#!/bin/bash

# 젯슨 오린 나노 시스템 테스트 및 문제 해결 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

echo "========================================="
echo "젯슨 오린 나노 시스템 테스트"
echo "========================================="

# 1. 시스템 정보 확인
echo "🖥️ 시스템 정보:"
echo "  OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d'"' -f2)"
echo "  커널: $(uname -r)"
echo "  아키텍처: $(uname -m)"

# 젯슨 모델 확인
if [ -f /etc/nv_tegra_release ]; then
    echo "  젯슨: $(cat /etc/nv_tegra_release)"
else
    echo "  ⚠️ 젯슨 환경이 아닐 수 있습니다"
fi

# 2. ROS 2 환경 확인
echo ""
echo "🤖 ROS 2 환경:"
if [ -n "$ROS_DISTRO" ]; then
    echo "  ✅ ROS_DISTRO: $ROS_DISTRO"
else
    echo "  ❌ ROS_DISTRO가 설정되지 않음"
    echo "  해결: source /opt/ros/foxy/setup.bash"
fi

# ROS 2 설치 확인
if command -v ros2 &> /dev/null; then
    echo "  ✅ ros2 명령어 사용 가능"
    ros2 --version 2>/dev/null || echo "  ⚠️ ros2 버전 정보 확인 실패"
else
    echo "  ❌ ros2 명령어 없음 - ROS 2 설치 필요"
fi

# 3. Python 환경 및 의존성 확인
echo ""
echo "🐍 Python 환경:"
echo "  Python: $(python3 --version)"
echo "  pip: $(pip3 --version | cut -d' ' -f2)"

# 핵심 Python 패키지 확인
echo ""
echo "📦 핵심 Python 패키지:"
python3 -c "
import sys
packages = [
    ('numpy', 'numpy'),
    ('cv2', 'opencv-python'),
    ('torch', 'torch'),
    ('ultralytics', 'ultralytics'),
    ('rclpy', 'rclpy')
]

for module, package in packages:
    try:
        mod = __import__(module)
        version = getattr(mod, '__version__', 'unknown')
        print(f'  ✅ {package}: {version}')
    except ImportError:
        print(f'  ❌ {package}: 설치 필요')
"

# 4. 빌드 환경 확인
echo ""
echo "🔨 빌드 환경:"
if command -v colcon &> /dev/null; then
    echo "  ✅ colcon 사용 가능"
else
    echo "  ❌ colcon 설치 필요"
    echo "  해결: sudo apt install python3-colcon-common-extensions"
fi

# 워크스페이스 확인
if [ -d "src" ] && [ -d "src/delivery_robot_description" ]; then
    echo "  ✅ ROS 2 워크스페이스 구조 확인"
    echo "  패키지 수: $(find src -name package.xml | wc -l)"
else
    echo "  ❌ ROS 2 워크스페이스 구조 문제"
fi

# 5. 하드웨어 리소스 확인
echo ""
echo "💾 하드웨어 리소스:"
echo "  메모리 사용량:"
free -h | grep -E "Mem|Swap" | sed 's/^/    /'

echo "  디스크 사용량:"
df -h / | tail -1 | awk '{print "    루트: " $3 "/" $2 " (" $5 " 사용)"}'

# GPU 확인 (젯슨)
if command -v nvidia-smi &> /dev/null; then
    echo "  GPU: 사용 가능"
else
    echo "  GPU: nvidia-smi 명령어 없음"
fi

# 6. 네트워크 확인
echo ""
echo "🌐 네트워크:"
if ping -c 1 8.8.8.8 >/dev/null 2>&1; then
    echo "  ✅ 인터넷 연결 정상"
else
    echo "  ❌ 인터넷 연결 문제"
fi

# 7. USB 카메라 확인
echo ""
echo "📹 카메라 장치:"
camera_count=$(ls /dev/video* 2>/dev/null | wc -l)
if [ $camera_count -gt 0 ]; then
    echo "  ✅ 카메라 장치 ${camera_count}개 감지:"
    ls /dev/video* | sed 's/^/    /'
else
    echo "  ❌ 카메라 장치 없음"
fi

# 8. 빌드 테스트 (선택적)
echo ""
echo "🧪 빌드 테스트:"
if [ -f "install/setup.bash" ]; then
    echo "  ✅ 이전 빌드 결과 존재"
    source install/setup.bash
    
    # 패키지 확인
    echo "  설치된 패키지:"
    ros2 pkg list | grep delivery_robot | sed 's/^/    /' || echo "    ❌ delivery_robot 패키지 없음"
else
    echo "  ⚠️ 빌드 결과 없음 - 먼저 빌드 실행 필요"
fi

# 9. 문제 해결 안내
echo ""
echo "========================================="
echo "🔧 일반적인 문제 해결:"
echo "========================================="
echo ""
echo "1. ROS 2 환경 설정 문제:"
echo "   source /opt/ros/foxy/setup.bash"
echo "   echo 'source /opt/ros/foxy/setup.bash' >> ~/.bashrc"
echo ""
echo "2. Python 의존성 문제:"
echo "   ./install_python_deps.sh"
echo ""
echo "3. 메모리 부족 문제:"
echo "   sudo swapoff -a && sudo swapon -a  # 스왑 재설정"
echo "   export MAKEFLAGS=\"-j1\"           # 단일 코어 빌드"
echo ""
echo "4. 권한 문제:"
echo "   sudo chown -R \$USER:USER ."
echo "   chmod +x *.sh"
echo ""
echo "5. 젯슨 성능 최적화:"
echo "   sudo jetson_clocks                 # 최대 성능"
echo "   sudo nvpmodel -m 0                # 최대 전력 모드"
echo ""
echo "6. 빌드 재시도:"
echo "   rm -rf build install log"
echo "   ./build_and_run_jetson.sh"

echo ""
echo "✨ 시스템 테스트 완료!"