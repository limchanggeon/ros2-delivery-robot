#!/bin/bash

# 젯슨에서 launch 파일 즉시 수동 설치 스크립트
# 젯슨에서 실행하세요: chmod +x quick_fix_launch.sh && ./quick_fix_launch.sh

echo "========================================="
echo "Launch 파일 즉시 수동 설치"
echo "========================================="

# 현재 경로 확인
if [ ! -d "src/delivery_robot_mission" ]; then
    echo "❌ ROS 2 워크스페이스 루트에서 실행하세요"
    exit 1
fi

echo "🔧 Launch 파일 수동 복사 중..."

# 모든 패키지의 launch 파일 복사
packages=("delivery_robot_mission" "delivery_robot_perception" "delivery_robot_security" "delivery_robot_navigation" "delivery_robot_description")

for pkg in "${packages[@]}"; do
    if [ -d "src/$pkg/launch" ]; then
        echo "📂 $pkg launch 파일 복사 중..."
        
        # install 디렉토리 생성
        mkdir -p "install/$pkg/share/$pkg/launch"
        
        # launch 파일 복사
        cp -v src/$pkg/launch/*.launch.py "install/$pkg/share/$pkg/launch/" 2>/dev/null || true
        
        # 복사된 파일 수 확인
        copied_files=$(find "install/$pkg/share/$pkg/launch" -name "*.launch.py" 2>/dev/null | wc -l)
        if [ $copied_files -gt 0 ]; then
            echo "  ✅ $copied_files 개 파일 복사 완료"
        else
            echo "  ⚠️ 복사할 launch 파일 없음"
        fi
    fi
done

echo ""
echo "🔧 설정 파일 복사 중..."

# 설정 파일도 복사
for pkg in "${packages[@]}"; do
    if [ -d "src/$pkg/config" ]; then
        echo "📂 $pkg config 파일 복사 중..."
        
        # install 디렉토리 생성
        mkdir -p "install/$pkg/share/$pkg/config"
        
        # config 파일 복사
        cp -v src/$pkg/config/*.yaml "install/$pkg/share/$pkg/config/" 2>/dev/null || true
        
        # 복사된 파일 수 확인
        copied_files=$(find "install/$pkg/share/$pkg/config" -name "*.yaml" 2>/dev/null | wc -l)
        if [ $copied_files -gt 0 ]; then
            echo "  ✅ $copied_files 개 파일 복사 완료"
        else
            echo "  ⚠️ 복사할 config 파일 없음"
        fi
    fi
done

echo ""
echo "🚀 환경 재설정 중..."
source install/setup.bash

echo ""
echo "✅ 수동 설치 완료!"
echo ""
echo "이제 다음 명령어로 테스트하세요:"
echo "ros2 launch delivery_robot_mission full_system_launch.py"
echo ""
echo "또는 개별 노드 테스트:"
echo "ros2 run delivery_robot_mission system_monitor_node"