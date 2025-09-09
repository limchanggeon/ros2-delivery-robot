#!/bin/bash

# 프로젝트 상태 점검 스크립트
# 작성자: 배달로봇팀
# 날짜: 2025-09-09

echo "========================================="
echo "ROS 2 배달 로봇 프로젝트 상태 점검"
echo "========================================="

# 1. 프로젝트 구조 확인
echo "📁 프로젝트 구조 확인..."
echo "✅ 워크스페이스 루트: $(pwd)"
echo "✅ src 폴더: $(ls -1 src/ | wc -l)개 패키지"

# 2. 패키지 목록 확인
echo ""
echo "📦 ROS 2 패키지 목록:"
for pkg in src/*/; do
    pkg_name=$(basename "$pkg")
    if [ -f "$pkg/package.xml" ]; then
        echo "  ✅ $pkg_name"
    else
        echo "  ❌ $pkg_name (package.xml 누락)"
    fi
done

# 3. 중요 파일 확인
echo ""
echo "📄 중요 파일 확인:"

# YOLOv8 모델
if [ -f "models/yolov8_best.pt" ]; then
    size=$(ls -lh models/yolov8_best.pt | awk '{print $5}')
    echo "  ✅ YOLOv8 모델: models/yolov8_best.pt ($size)"
else
    echo "  ❌ YOLOv8 모델 누락: models/yolov8_best.pt"
fi

# 설정 파일들
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
        echo "  ✅ $(basename "$file")"
    else
        echo "  ❌ $(basename "$file") 누락: $file"
    fi
done

# 4. Python 의존성 확인
echo ""
echo "🐍 Python 의존성 확인:"
python3 -c "
import sys
packages = [
    ('numpy', 'numpy'),
    ('cv2', 'opencv-python'),
    ('torch', 'torch'),
    ('ultralytics', 'ultralytics'),
    ('requests', 'requests'),
    ('geopy', 'geopy'),
    ('serial', 'pyserial'),
    ('cryptography', 'cryptography'),
    ('qrcode', 'qrcode'),
    ('pyzbar', 'pyzbar')
]

missing = []
for module, package in packages:
    try:
        __import__(module)
        print(f'  ✅ {package}')
    except ImportError:
        print(f'  ❌ {package} (설치 필요)')
        missing.append(package)

if missing:
    print(f'\n❌ 누락된 패키지: {len(missing)}개')
    print('설치 명령: pip3 install --user ' + ' '.join(missing))
else:
    print('\n✅ 모든 Python 의존성이 설치되어 있습니다!')
"

# 5. Git 상태 확인
echo ""
echo "📝 Git 상태:"
if [ -d ".git" ]; then
    echo "  ✅ Git 저장소 초기화됨"
    echo "  📍 현재 브랜치: $(git branch --show-current)"
    
    if git diff --quiet && git diff --cached --quiet; then
        echo "  ✅ 모든 변경사항이 커밋됨"
    else
        echo "  ⚠️  커밋되지 않은 변경사항이 있습니다"
        echo "  수정된 파일 수: $(git status --porcelain | wc -l)"
    fi
else
    echo "  ❌ Git 저장소가 초기화되지 않음"
fi

# 6. 디스크 사용량 확인
echo ""
echo "💾 디스크 사용량:"
total_size=$(du -sh . | cut -f1)
models_size=$(du -sh models/ 2>/dev/null | cut -f1 || echo "0B")
echo "  📊 전체 프로젝트: $total_size"
echo "  🤖 모델 파일: $models_size"

echo ""
echo "========================================="
echo "✨ 상태 점검 완료!"
echo "========================================="

# 7. 다음 단계 안내
echo ""
echo "🚀 다음 단계:"
echo "1. Python 의존성 설치: ./install_python_deps.sh"
echo "2. 프로젝트 빌드 (Linux): ./build_and_run.sh"
echo "3. 프로젝트 빌드 (macOS): ./build_and_run_macos.sh"
echo "4. Git 커밋 및 푸시: git add -A && git commit -m 'Update' && git push"