#!/bin/bash

# 젯슨 빌드 후 launch 파일 설치 문제 해결 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

echo "========================================="
echo "젯슨 Launch 파일 설치 문제 해결"
echo "========================================="

# ROS 2 환경 설정
if [ -z "$ROS_DISTRO" ]; then
    echo "🔧 ROS 2 환경 설정 중..."
    source /opt/ros/foxy/setup.bash
fi

# 현재 디렉토리 확인
if [ ! -d "src/delivery_robot_mission" ]; then
    echo "❌ ROS 2 워크스페이스 루트에서 실행하세요"
    exit 1
fi

echo "📂 Launch 파일 설치 상태 확인..."

# install 디렉토리가 있는지 확인
if [ ! -d "install" ]; then
    echo "❌ install 디렉토리 없음 - 먼저 빌드가 필요합니다"
    echo "해결: ./build_and_run_jetson.sh"
    exit 1
fi

# 각 패키지의 launch 파일 설치 상태 확인
packages=("delivery_robot_mission" "delivery_robot_perception" "delivery_robot_security" "delivery_robot_navigation" "delivery_robot_description")

for pkg in "${packages[@]}"; do
    echo ""
    echo "📦 $pkg 패키지 확인..."
    
    # 소스 launch 디렉토리 확인
    src_launch_dir="src/$pkg/launch"
    if [ -d "$src_launch_dir" ]; then
        echo "  ✅ 소스 launch 디렉토리 존재: $src_launch_dir"
        launch_files=$(find "$src_launch_dir" -name "*.launch.py" | wc -l)
        echo "  📄 Launch 파일 수: $launch_files"
        find "$src_launch_dir" -name "*.launch.py" | sed 's/^/    /'
    else
        echo "  ⚠️ 소스 launch 디렉토리 없음: $src_launch_dir"
        continue
    fi
    
    # 설치된 launch 디렉토리 확인
    install_launch_dir="install/$pkg/share/$pkg/launch"
    if [ -d "$install_launch_dir" ]; then
        echo "  ✅ 설치된 launch 디렉토리 존재: $install_launch_dir"
        installed_files=$(find "$install_launch_dir" -name "*.launch.py" | wc -l)
        echo "  📄 설치된 Launch 파일 수: $installed_files"
        find "$install_launch_dir" -name "*.launch.py" | sed 's/^/    /'
        
        # 파일 수가 다르면 문제
        if [ "$launch_files" -ne "$installed_files" ]; then
            echo "  ❌ 설치된 파일 수가 소스와 다름!"
            echo "  🔧 패키지 재빌드 필요"
        fi
    else
        echo "  ❌ 설치된 launch 디렉토리 없음: $install_launch_dir"
        echo "  🔧 패키지 재빌드 필요"
    fi
done

echo ""
echo "========================================="
echo "🔧 문제 해결 방법"
echo "========================================="
echo ""
echo "1. 전체 재빌드 (권장):"
echo "   rm -rf build install log"
echo "   ./build_and_run_jetson.sh"
echo ""
echo "2. 특정 패키지만 재빌드:"
echo "   colcon build --packages-select delivery_robot_mission --symlink-install"
echo ""
echo "3. 캐시 정리 후 재빌드:"
echo "   rm -rf ~/.ros/log*"
echo "   rm -rf build install log"
echo "   source /opt/ros/foxy/setup.bash"
echo "   colcon build --symlink-install"
echo ""
echo "4. Launch 파일 수동 복사 (임시 해결):"
echo "   mkdir -p install/delivery_robot_mission/share/delivery_robot_mission/launch"
echo "   cp src/delivery_robot_mission/launch/*.launch.py install/delivery_robot_mission/share/delivery_robot_mission/launch/"
echo ""
echo "5. 환경 재설정:"
echo "   source install/setup.bash"
echo "   ros2 launch delivery_robot_mission full_system_launch.py"

echo ""
echo "========================================="
echo "🎯 빠른 테스트"
echo "========================================="
echo ""
echo "# 패키지 확인"
echo "ros2 pkg list | grep delivery_robot"
echo ""
echo "# Launch 파일 확인"
echo "ros2 pkg prefix delivery_robot_mission"
echo "find \$(ros2 pkg prefix delivery_robot_mission) -name '*.launch.py'"
echo ""
echo "# 직접 실행 테스트"
echo "python3 src/delivery_robot_mission/launch/full_system_launch.py"

echo ""
echo "✨ 진단 완료!"