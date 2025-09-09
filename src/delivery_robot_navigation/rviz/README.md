# RViz 설정 파일 디렉토리

이 디렉토리는 RViz 시각화 설정 파일들을 포함합니다.

## 설정 파일

- `delivery_robot_nav.rviz`: 네비게이션용 RViz 설정
- `delivery_robot_full.rviz`: 전체 시스템용 RViz 설정

## 사용법

RViz 설정 파일을 사용하여 시각화를 시작:

```bash
rviz2 -d src/delivery_robot_navigation/rviz/delivery_robot_nav.rviz
```

## 주의사항

실제 RViz 설정 파일은 RViz를 실행한 후 원하는 설정으로 구성하고 저장해야 합니다.