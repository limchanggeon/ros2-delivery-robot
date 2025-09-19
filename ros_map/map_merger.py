#!/usr/bin/env python3
"""
네이버 지도 4분할 이미지를 합치고 ROS용 지도로 변환하는 스크립트
사용법: python3 map_merger.py
"""

import cv2
import numpy as np
import os
import yaml
from PIL import Image
import matplotlib.pyplot as plt

class MapMerger:
    def __init__(self, map_folder="/Users/limchang-geon/Desktop/capston_project/ros_map"):
        self.map_folder = map_folder
        self.output_map_name = "merged_naver_map"
        
    def load_images(self):
        """4개의 분할 이미지 로드"""
        images = {}
        for i in range(1, 5):
            img_path = os.path.join(self.map_folder, f"{i}.png")
            if os.path.exists(img_path):
                img = cv2.imread(img_path)
                images[i] = img
                print(f"✅ {i}.png 로드됨: {img.shape}")
            else:
                print(f"❌ {img_path} 파일을 찾을 수 없습니다.")
                return None
        return images
    
    def determine_layout(self, images):
        """이미지 크기를 보고 배치 방식 결정"""
        print("\n📐 이미지 배치 분석 중...")
        
        # 네이버 지도 스크롤 캡처를 고려한 자동 판단
        print("🗂️ 네이버 지도 스크롤 캡처 감지됨 - 세로 배치(4x1)로 자동 설정")
        print("   • 1.png: 맨 위")
        print("   • 2.png: 위에서 두 번째") 
        print("   • 3.png: 위에서 세 번째")
        print("   • 4.png: 맨 아래")
        
        return "4x1_scroll"
    
    def resize_images_to_same_size(self, images):
        """모든 이미지를 동일한 크기로 리사이즈"""
        print("📏 이미지 크기 통일 중...")
        
        # 가장 작은 공통 크기 찾기
        min_height = min(img.shape[0] for img in images.values())
        min_width = min(img.shape[1] for img in images.values())
        
        print(f"   통일 크기: {min_height} x {min_width}")
        
        resized_images = {}
        for i, img in images.items():
            if img.shape[:2] != (min_height, min_width):
                resized = cv2.resize(img, (min_width, min_height))
                resized_images[i] = resized
                print(f"   {i}.png: {img.shape} → {resized.shape}")
            else:
                resized_images[i] = img
                print(f"   {i}.png: 크기 변경 없음 {img.shape}")
                
        return resized_images
    
    def merge_images_2x2(self, images):
        """2x2 격자로 이미지 합치기"""
        print("🔧 2x2 격자로 이미지 합치는 중...")
        
        # 이미지 크기 통일
        resized_images = self.resize_images_to_same_size(images)
        
        # 상단 행 (1, 2)
        top_row = cv2.hconcat([resized_images[1], resized_images[2]])
        # 하단 행 (3, 4)  
        bottom_row = cv2.hconcat([resized_images[3], resized_images[4]])
        # 전체 합치기
        merged = cv2.vconcat([top_row, bottom_row])
        
        return merged
    
    def merge_images_1x4(self, images):
        """1x4 가로 배치로 이미지 합치기"""
        print("🔧 1x4 가로 배치로 이미지 합치는 중...")
        
        # 이미지 크기 통일
        resized_images = self.resize_images_to_same_size(images)
        
        merged = cv2.hconcat([resized_images[1], resized_images[2], resized_images[3], resized_images[4]])
        return merged
    
    def merge_images_4x1_scroll(self, images):
        """4x1 세로 배치로 스크롤 캡처 이미지들을 겹침 영역 제거하면서 합치기"""
        print("🔧 스크롤 캡처 이미지를 세로로 연결 중...")
        
        # 모든 이미지를 같은 너비로 맞춤
        target_width = min(img.shape[1] for img in images.values())
        print(f"   통일 너비: {target_width}px")
        
        resized_images = {}
        for i, img in images.items():
            if img.shape[1] != target_width:
                # 너비만 맞추고 비율 유지
                height = int(img.shape[0] * (target_width / img.shape[1]))
                resized = cv2.resize(img, (target_width, height))
                resized_images[i] = resized
                print(f"   {i}.png: {img.shape} → {resized.shape}")
            else:
                resized_images[i] = img
                print(f"   {i}.png: 크기 변경 없음 {img.shape}")
        
        # 겹침 영역 자동 감지 및 제거
        print("\n🔍 겹침 영역 감지 중...")
        processed_images = [resized_images[1]]  # 첫 번째 이미지는 그대로
        
        for i in range(2, 5):  # 2, 3, 4번 이미지 처리
            current_img = resized_images[i]
            prev_combined = processed_images[-1] if len(processed_images) == 1 else self.combine_images_vertically(processed_images)
            
            # 겹침 영역 찾기 (아래쪽 일부와 위쪽 일부 비교)
            overlap_height = self.find_overlap_height(prev_combined, current_img)
            print(f"   {i}.png: {overlap_height}px 겹침 영역 감지")
            
            if overlap_height > 0:
                # 겹침 부분 제거
                cropped_img = current_img[overlap_height:, :]
                processed_images.append(cropped_img)
                print(f"   {i}.png: 겹침 제거 후 크기 {cropped_img.shape}")
            else:
                processed_images.append(current_img)
                print(f"   {i}.png: 겹침 없음")
        
        # 최종 세로 연결
        final_merged = self.combine_images_vertically(processed_images)
        print(f"✅ 최종 연결된 이미지 크기: {final_merged.shape}")
        
        return final_merged
    
    def find_overlap_height(self, img1, img2, search_height=100):
        """두 이미지 간의 겹침 높이를 찾는 함수"""
        if img1.shape[1] != img2.shape[1]:
            return 0
        
        img1_bottom = img1[-search_height:, :]  # 첫 번째 이미지의 아래쪽
        img2_top = img2[:search_height, :]      # 두 번째 이미지의 위쪽
        
        best_overlap = 0
        best_score = float('inf')
        
        # 가능한 겹침 높이들을 테스트
        for overlap in range(1, min(search_height, img1.shape[0], img2.shape[0]) - 1):
            try:
                # 비교할 영역 추출
                region1 = img1_bottom[-overlap:, :]
                region2 = img2_top[:overlap, :]
                
                if region1.shape == region2.shape:
                    # 차이 계산 (낮을수록 더 유사)
                    diff = cv2.absdiff(region1, region2)
                    score = np.mean(diff)
                    
                    if score < best_score:
                        best_score = score
                        best_overlap = overlap
                        
            except:
                continue
        
        # 임계값보다 유사도가 높은 경우만 겹침으로 판단
        similarity_threshold = 30  # 조정 가능
        if best_score < similarity_threshold:
            return best_overlap
        else:
            return 0
    
    def combine_images_vertically(self, image_list):
        """이미지 리스트를 세로로 연결"""
        if len(image_list) == 1:
            return image_list[0]
        
        result = image_list[0]
        for img in image_list[1:]:
            result = cv2.vconcat([result, img])
        return result
    
    def preprocess_for_ros(self, image):
        """ROS 지도 형식으로 전처리"""
        print("🎨 ROS 지도 형식으로 변환 중...")
        
        # 그레이스케일 변환
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image.copy()
        
        # 대비 향상
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(gray)
        
        # ROS 표준 형식으로 변환
        # 흰색(255) = 자유 공간, 검은색(0) = 장애물, 회색(128) = 미지 영역
        
        print("\n🎯 지도 변환 옵션:")
        print("1. 자동 임계값 (Otsu)")
        print("2. 수동 임계값 설정") 
        print("3. 적응적 임계값")
        
        method = input("변환 방법 선택 (1-3): ").strip()
        
        if method == "2":
            # 수동 임계값
            threshold = int(input("임계값 입력 (0-255, 권장: 128): "))
            _, binary = cv2.threshold(enhanced, threshold, 255, cv2.THRESH_BINARY)
        elif method == "3":
            # 적응적 임계값
            binary = cv2.adaptiveThreshold(enhanced, 255, cv2.THRESH_ADAPTIVE_MEAN_C, cv2.THRESH_BINARY, 11, 2)
        else:
            # 자동 임계값 (Otsu)
            _, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        return binary
    
    def calculate_resolution(self):
        """축척비 이미지를 보고 해상도 계산"""
        scale_img_path = os.path.join(self.map_folder, "축척비.png")
        
        if os.path.exists(scale_img_path):
            print("\n📏 축척비 이미지를 발견했습니다!")
            
            # 축척비 이미지 표시
            scale_img = cv2.imread(scale_img_path)
            if scale_img is not None:
                print("축척비 이미지를 확인하고 실제 거리를 측정하세요.")
                
                # 사용자에게 축척 정보 입력받기
                print("\n📐 축척 정보 입력:")
                print("예: '100m', '50m', '1km' 등")
                scale_distance = input("축척바가 나타내는 실제 거리: ").strip()
                
                if scale_distance:
                    # 거리에서 숫자 추출
                    import re
                    numbers = re.findall(r'\d+', scale_distance)
                    if numbers:
                        distance_value = float(numbers[0])
                        
                        # 단위 확인
                        if 'km' in scale_distance.lower():
                            distance_meters = distance_value * 1000
                        else:
                            distance_meters = distance_value
                        
                        # 축척바 픽셀 길이 입력받기
                        scale_pixels = float(input("축척바의 픽셀 길이: "))
                        
                        resolution = distance_meters / scale_pixels
                        print(f"✅ 계산된 해상도: {resolution:.4f} meters/pixel")
                        return resolution
        
        # 기본값 설정
        print("⚠️ 축척 정보가 없어 기본 해상도를 사용합니다.")
        default_resolution = float(input("해상도 입력 (meters/pixel, 기본값 0.1): ") or "0.1")
        return default_resolution
    
    def create_yaml_file(self, resolution, image_filename):
        """ROS 지도용 YAML 파일 생성"""
        yaml_filename = f"{self.output_map_name}.yaml"
        yaml_path = os.path.join(self.map_folder, yaml_filename)
        
        map_data = {
            'image': f"./{image_filename}",
            'resolution': resolution,
            'origin': [0.0, 0.0, 0.0],  # 지도 원점
            'negate': 0,  # 0 = 흰색이 자유공간
            'occupied_thresh': 0.65,  # 장애물로 간주할 임계값
            'free_thresh': 0.25       # 자유공간으로 간주할 임계값
        }
        
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(map_data, f, default_flow_style=False)
        
        print(f"✅ YAML 파일 생성됨: {yaml_path}")
        return yaml_path
    
    def process(self):
        """전체 처리 과정 실행"""
        print("🗺️ 네이버 지도 4분할 이미지 합치기 시작!")
        print("=" * 60)
        
        # 1. 이미지 로드
        images = self.load_images()
        if images is None:
            return False
        
        # 2. 배치 방식 결정
        layout = self.determine_layout(images)
        
        # 3. 이미지 합치기
        if layout == "2x2":
            merged = self.merge_images_2x2(images)
        elif layout == "1x4":
            merged = self.merge_images_1x4(images)
        elif layout == "4x1":
            merged = self.merge_images_4x1(images)
        elif layout == "4x1_scroll":
            merged = self.merge_images_4x1_scroll(images)
        else:
            merged = self.merge_images_4x1_scroll(images)  # 기본값
        
        print(f"✅ 합친 이미지 크기: {merged.shape}")
        
        # 4. 원본 합친 이미지 저장
        merged_original_path = os.path.join(self.map_folder, f"{self.output_map_name}_original.png")
        cv2.imwrite(merged_original_path, merged)
        print(f"💾 원본 합친 이미지 저장됨: {merged_original_path}")
        
        # 5. ROS 형식으로 전처리
        processed = self.preprocess_for_ros(merged)
        
        # 6. ROS 지도 파일 저장
        map_image_path = os.path.join(self.map_folder, f"{self.output_map_name}.pgm")
        cv2.imwrite(map_image_path, processed)
        print(f"💾 ROS 지도 이미지 저장됨: {map_image_path}")
        
        # 7. 해상도 계산
        resolution = self.calculate_resolution()
        
        # 8. YAML 파일 생성
        yaml_path = self.create_yaml_file(resolution, f"{self.output_map_name}.pgm")
        
        # 9. 미리보기 생성
        self.create_preview(merged, processed)
        
        print("\n" + "=" * 60)
        print("🎉 지도 생성 완료!")
        print(f"📁 생성된 파일들:")
        print(f"   • 원본 합친 이미지: {self.output_map_name}_original.png")
        print(f"   • ROS 지도 이미지: {self.output_map_name}.pgm") 
        print(f"   • 지도 설정 파일: {self.output_map_name}.yaml")
        print(f"   • 미리보기: {self.output_map_name}_preview.png")
        print("\n🚀 사용 방법:")
        print(f"   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path}")
        print("   또는")
        print(f"   ./scripts/quick_start_gps_system.sh {yaml_path}")
        
        return True
    
    def create_preview(self, original, processed):
        """원본과 처리된 이미지 비교 미리보기 생성"""
        fig, axes = plt.subplots(1, 2, figsize=(15, 8))
        
        # 원본 이미지
        if len(original.shape) == 3:
            original_rgb = cv2.cvtColor(original, cv2.COLOR_BGR2RGB)
        else:
            original_rgb = original
        axes[0].imshow(original_rgb)
        axes[0].set_title('원본 합친 이미지', fontsize=14)
        axes[0].axis('off')
        
        # 처리된 이미지
        axes[1].imshow(processed, cmap='gray')
        axes[1].set_title('ROS 지도 (흰색=자유공간, 검은색=장애물)', fontsize=14)
        axes[1].axis('off')
        
        plt.tight_layout()
        preview_path = os.path.join(self.map_folder, f"{self.output_map_name}_preview.png")
        plt.savefig(preview_path, dpi=150, bbox_inches='tight')
        plt.close()
        
        print(f"📸 미리보기 이미지 저장됨: {preview_path}")

def main():
    merger = MapMerger()
    success = merger.process()
    
    if success:
        print("\n✨ 모든 작업이 성공적으로 완료되었습니다!")
    else:
        print("\n❌ 작업 중 오류가 발생했습니다.")

if __name__ == "__main__":
    main()