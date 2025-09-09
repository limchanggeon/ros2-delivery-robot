# GitHub 저장소 설정 가이드

이 문서는 프로젝트를 GitHub에 업로드하는 방법을 설명합니다.

## 1. GitHub에서 새 저장소 생성

1. [GitHub.com](https://github.com)에 로그인
2. 우상단의 "+" 버튼 클릭 → "New repository" 선택
3. 저장소 설정:
   - **Repository name**: `ros2-delivery-robot` (또는 원하는 이름)
   - **Description**: `ROS 2 자율주행 배송 로봇 시스템 - YOLOv8 객체 인식, GPS 네비게이션, QR 인증`
   - **Public** 또는 **Private** 선택
   - ✅ **Add a README file**: 체크 해제 (이미 README.md가 있음)
   - ✅ **Add .gitignore**: 체크 해제 (이미 .gitignore가 있음)
   - ✅ **Choose a license**: MIT License 선택 (권장)

4. "Create repository" 클릭

## 2. 로컬 저장소와 GitHub 연결

터미널에서 다음 명령어를 실행하세요:

```bash
# GitHub 저장소와 연결 (YOUR_USERNAME을 실제 GitHub 사용자명으로 변경)
git remote add origin https://github.com/limchanggeon/ros2-delivery-robot.git

# 원격 저장소에 푸시
git push -u origin main
```

## 3. Git 사용자 정보 설정 (선택사항)

만약 Git 사용자 정보가 설정되지 않았다면:

```bash
# 전역 사용자 정보 설정
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# 이전 커밋의 작성자 정보 수정
git commit --amend --reset-author
```

## 4. 대용량 파일 처리 (Git LFS)

YOLOv8 모델 파일(.pt)이 100MB를 초과하는 경우 Git LFS를 사용해야 합니다:

```bash
# Git LFS 설치 (Ubuntu/macOS)
# Ubuntu: sudo apt install git-lfs
# macOS: brew install git-lfs

# Git LFS 초기화
git lfs install

# .pt 파일을 LFS로 추적
git lfs track "*.pt"

# .gitattributes 파일 추가
git add .gitattributes

# 변경사항 커밋
git commit -m "Add Git LFS tracking for model files"

# 푸시
git push origin main
```

## 5. 저장소 완성 확인

GitHub 페이지에서 다음을 확인하세요:

- ✅ 모든 파일이 정상적으로 업로드됨
- ✅ README.md가 제대로 표시됨
- ✅ 모델 파일(yolov8_best.pt)이 업로드됨
- ✅ 프로젝트 구조가 올바르게 표시됨

## 6. 추가 설정 (선택사항)

### 브랜치 보호 규칙 설정
- Settings → Branches → Add rule
- main 브랜치에 대한 보호 규칙 설정

### 이슈 템플릿 생성
- Settings → Features → Issues → Set up templates

### GitHub Actions 설정
- Actions 탭에서 CI/CD 워크플로우 설정

## 7. 협업자 추가

- Settings → Manage access → Invite a collaborator

## 문제 해결

### 푸시 실패 시
```bash
# 원격 저장소 상태 확인
git remote -v

# 강제 푸시 (주의: 기존 데이터 손실 가능)
git push --force origin main
```

### 대용량 파일 오류 시
```bash
# 파일 크기 확인
find . -type f -size +100M

# Git LFS 사용하여 대용량 파일 처리 (위의 4번 참조)
```

저장소가 성공적으로 생성되면 다음 URL에서 확인할 수 있습니다:
`https://github.com/YOUR_USERNAME/ros2-delivery-robot`