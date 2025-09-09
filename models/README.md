# 모델 디렉토리

이 디렉토리는 ROS 2 배송 로봇 프로젝트에서 사용되는 딥러닝 모델들을 포함합니다.

## YOLOv8 모델 (yolov8_best.pt)

### 모델 정보
- **모델 타입**: YOLOv8 (You Only Look Once v8)
- **용도**: 실시간 객체 탐지 및 인식
- **파일명**: `yolov8_best.pt`
- **프레임워크**: PyTorch

### 사용 용도
이 모델은 배송 로봇의 인식 시스템에서 다음과 같은 객체들을 탐지하는데 사용됩니다:
- 사람 (person)
- 차량 (car, truck, bus)
- 장애물 (barrier, cone)
- 신호등 (traffic light)
- 기타 도로 관련 객체들

### 사용 방법
모델은 `delivery_robot_perception` 패키지의 `yolo_inference_node.py`에서 자동으로 로드됩니다.

```python
# 모델 로드 예시
from ultralytics import YOLO
model = YOLO('/path/to/models/yolov8_best.pt')
```

### 성능 최적화
- GPU 사용 시 CUDA 가속 지원
- CPU 환경에서도 실시간 처리 가능
- 입력 이미지 크기: 640x640 (기본값)

### 모델 업데이트
새로운 모델로 교체할 때는 기존 파일을 백업하고 같은 이름으로 새 모델을 저장하세요.

```bash
# 백업
cp yolov8_best.pt yolov8_best_backup_$(date +%Y%m%d).pt

# 새 모델로 교체
cp /path/to/new_model.pt yolov8_best.pt
```