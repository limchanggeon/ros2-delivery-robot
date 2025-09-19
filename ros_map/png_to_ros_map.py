#!/usr/bin/env python3
"""
완성된 PNG 지도를 ROS 지도로 변환하고 최적화하는 스크립트
사용법: python3 png_to_ros_map.py
"""

import cv2
import numpy as np
import os
import yaml
from PIL import Image, ImageEnhance
import matplotlib.pyplot as plt

class PngToRosMap:
    def __init__(self, map_folder="/Users/limchang-geon/Desktop/capston_project/ros_map", auto_mode=False):
        self.map_folder = map_folder
        self.input_file = "map.png"
        self.output_name = "optimized_map"
        self.auto_mode = auto_mode  # 비대화형 모드
        
    def load_and_analyze_image(self):
        """PNG 이미지 로드 및 분석"""
        img_path = os.path.join(self.map_folder, self.input_file)
        
        if not os.path.exists(img_path):
            print(f"❌ {img_path} 파일을 찾을 수 없습니다.")
            return None
            
        img = cv2.imread(img_path)
        if img is None:
            print(f"❌ 이미지를 읽을 수 없습니다: {img_path}")
            return None
            
        print(f"✅ 이미지 로드됨: {img.shape}")
        print(f"   크기: {img.shape[1]} x {img.shape[0]} pixels")
        print(f"   파일 크기: {os.path.getsize(img_path) / (1024*1024):.1f} MB")
        
        return img
    
    def optimize_image_size(self, img):
        """이미지 크기 최적화 (ROS 성능을 위해)"""
        height, width = img.shape[:2]
        
        print(f"\n📐 이미지 크기 최적화 분석:")
        print(f"   현재 크기: {width} x {height}")
        
        # ROS에서 권장하는 최대 크기 (메모리 및 성능 고려)
        max_dimension = 4000
        
        if max(width, height) > max_dimension:
            # 비율 유지하면서 크기 축소
            scale = max_dimension / max(width, height)
            new_width = int(width * scale)
            new_height = int(height * scale)
            
            print(f"   크기 축소 필요: {new_width} x {new_height} (scale: {scale:.3f})")
            
            # 고품질 리사이즈
            optimized = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)
            print(f"   ✅ 최적화 완료: {optimized.shape}")
            return optimized
        else:
            print(f"   ✅ 크기 최적화 불필요 (적정 크기)")
            return img
    
    def enhance_map_quality(self, img):
        """지도 품질 향상"""
        print(f"\n🎨 지도 품질 향상 중...")
        
        # PIL로 변환 (더 나은 이미지 처리를 위해)
        img_pil = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
        
        # 대비 향상
        enhancer = ImageEnhance.Contrast(img_pil)
        enhanced = enhancer.enhance(1.2)  # 20% 대비 증가
        
        # 선명도 향상
        enhancer = ImageEnhance.Sharpness(enhanced)
        enhanced = enhancer.enhance(1.1)  # 10% 선명도 증가
        
        # OpenCV로 다시 변환
        enhanced_cv = cv2.cvtColor(np.array(enhanced), cv2.COLOR_RGB2BGR)
        
        print(f"   ✅ 품질 향상 완료")
        return enhanced_cv
    
    def convert_to_ros_format(self, img):
        """ROS 지도 형식으로 변환 (개선된 버전)"""
        print(f"\n🗺️ ROS 지도 형식으로 변환 중...")
        
        # 그레이스케일 변환
        if len(img.shape) == 3:
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        else:
            gray = img.copy()
        
        # 히스토그램 분석으로 이미지 특성 파악
        hist = cv2.calcHist([gray], [0], None, [256], [0, 256])
        
        # 밝은 픽셀과 어두운 픽셀의 비율 확인
        dark_pixels = np.sum(hist[:128])
        bright_pixels = np.sum(hist[128:])
        
        print(f"   📊 이미지 분석:")
        print(f"      어두운 픽셀: {dark_pixels/1000:.0f}K ({dark_pixels/(dark_pixels+bright_pixels)*100:.1f}%)")
        print(f"      밝은 픽셀: {bright_pixels/1000:.0f}K ({bright_pixels/(dark_pixels+bright_pixels)*100:.1f}%)")
        
        # 노이즈 제거
        denoised = cv2.medianBlur(gray, 3)
        
        print(f"\n🎯 지도 이진화 방법 선택:")
        print(f"1. 자동 임계값 (Otsu) - 권장")
        print(f"2. 적응적 임계값 - 조명 변화가 많은 경우")
        print(f"3. 수동 임계값 - 직접 조정")
        print(f"4. 고급 분석 모드 - 네이버 지도 최적화")
        
        if self.auto_mode:
            choice = "4"
            print(f"자동 모드: 고급 분석 모드 선택됨")
        else:
            choice = input("변환 방법 선택 (1-4): ").strip() or "4"
        
        if choice == "4":
            # 네이버 지도에 최적화된 고급 처리
            binary = self.advanced_naver_map_processing(denoised)
        elif choice == "2":
            # 적응적 임계값
            binary = cv2.adaptiveThreshold(
                denoised, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, 
                cv2.THRESH_BINARY, 11, 2
            )
            print(f"   ✅ 적응적 임계값 적용")
        elif choice == "3":
            # 수동 임계값
            threshold = int(input("임계값 입력 (0-255, 기본값: 128): ") or "128")
            _, binary = cv2.threshold(denoised, threshold, 255, cv2.THRESH_BINARY)
            print(f"   ✅ 수동 임계값 적용: {threshold}")
        else:
            # 자동 임계값 (Otsu)
            threshold, binary = cv2.threshold(denoised, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
            print(f"   ✅ 자동 임계값 (Otsu) 적용: {threshold}")
        
        # 색상 반전 검사 (네이버 지도는 보통 흰 배경에 검은 도로)
        white_pixels = np.sum(binary == 255)
        black_pixels = np.sum(binary == 0)
        
        print(f"\n   🔍 이진화 결과 분석:")
        print(f"      흰색 픽셀: {white_pixels/1000:.0f}K ({white_pixels/(white_pixels+black_pixels)*100:.1f}%)")
        print(f"      검은색 픽셀: {black_pixels/1000:.0f}K ({black_pixels/(white_pixels+black_pixels)*100:.1f}%)")
        
        # 대부분이 흰색이라면 (90% 이상), 색상 반전이 필요할 수 있음
        if white_pixels / (white_pixels + black_pixels) > 0.9:
            print(f"   ⚠️  대부분이 흰색으로 변환됨 - 색상 반전을 고려해야 합니다")
            invert = input("   색상을 반전하시겠습니까? (Y/n): ").strip().lower()
            if invert != 'n':
                binary = cv2.bitwise_not(binary)
                print(f"   🔄 색상 반전 적용됨")
        
        # 모폴로지 연산으로 노이즈 제거 및 구멍 메우기
        kernel = np.ones((3,3), np.uint8)
        
        # 열림 연산 (작은 노이즈 제거)
        opened = cv2.morphologyEx(binary, cv2.MORPH_OPEN, kernel, iterations=1)
        
        # 닫힘 연산 (작은 구멍 메우기)  
        closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, kernel, iterations=1)
        
        print(f"   ✅ 노이즈 제거 및 후처리 완료")
        
        return closed
    
    def advanced_naver_map_processing(self, gray_img):
        """네이버 지도에 특화된 고급 처리"""
        print(f"   🚀 네이버 지도 최적화 처리 중...")
        
        # 1. 가우시안 블러로 노이즈 제거
        blurred = cv2.GaussianBlur(gray_img, (3, 3), 0)
        
        # 2. 대비 향상 (CLAHE)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        enhanced = clahe.apply(blurred)
        
        # 3. 다중 임계값 테스트
        thresholds = []
        methods = [
            ("Otsu", cv2.THRESH_OTSU),
            ("Triangle", cv2.THRESH_TRIANGLE),
            ("Mean", None)  # 평균값 사용
        ]
        
        for name, method in methods:
            if method is None:
                # 평균값 임계값
                mean_val = np.mean(enhanced)
                thresh_val = mean_val
                _, binary = cv2.threshold(enhanced, thresh_val, 255, cv2.THRESH_BINARY)
            else:
                thresh_val, binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + method)
            
            # 검은 픽셀 비율 계산
            black_ratio = np.sum(binary == 0) / binary.size
            thresholds.append((name, thresh_val, black_ratio, binary))
            print(f"      {name}: 임계값={thresh_val:.1f}, 검은픽셀={black_ratio*100:.1f}%")
        
        # 4. 최적의 임계값 선택 (검은 픽셀이 5-30% 범위인 것)
        best_binary = None
        best_score = float('inf')
        
        for name, thresh_val, black_ratio, binary in thresholds:
            # 이상적인 검은픽셀 비율: 15% (도로, 건물 윤곽 등)
            score = abs(black_ratio - 0.15)
            if score < best_score and 0.05 <= black_ratio <= 0.35:
                best_score = score
                best_binary = binary
                best_method = name
                print(f"      ✅ {name} 방법 선택됨 (검은픽셀: {black_ratio*100:.1f}%)")
        
        if best_binary is None:
            print(f"      ⚠️  최적 임계값 없음 - Otsu 방법 사용")
            _, best_binary = cv2.threshold(enhanced, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # 5. 에지 기반 보정 (도로 윤곽선 강화)
        edges = cv2.Canny(enhanced, 50, 150)
        
        # 에지를 장애물로 추가 (도로 경계선)
        combined = cv2.bitwise_or(255 - best_binary, edges)
        final_binary = 255 - combined
        
        print(f"   ✅ 고급 처리 완료")
        return final_binary
    
    def calculate_resolution_interactive(self):
        """사용자 입력으로 해상도 계산"""
        print(f"\n📏 지도 해상도 설정:")
        
        if self.auto_mode:
            print(f"자동 모드: 기본 해상도 0.3 m/pixel 사용 (1.2km 지도용)")
            return 0.3
        
        print(f"축척비 정보나 실제 거리를 알고 있는 구간이 있나요?")
        
        has_scale = input("축척 정보가 있습니까? (y/N): ").strip().lower() == 'y'
        
        if has_scale:
            print(f"\n실제 거리를 알고 있는 두 지점 간의 정보를 입력하세요:")
            real_distance = input("실제 거리 (예: 100m, 50m, 1km): ").strip()
            pixel_distance = input("지도상 픽셀 거리: ").strip()
            
            try:
                # 거리에서 숫자 추출
                import re
                numbers = re.findall(r'\d+\.?\d*', real_distance)
                if numbers:
                    distance_value = float(numbers[0])
                    
                    # 단위 확인
                    if 'km' in real_distance.lower():
                        distance_meters = distance_value * 1000
                    elif 'cm' in real_distance.lower():
                        distance_meters = distance_value / 100
                    else:
                        distance_meters = distance_value
                    
                    pixel_dist = float(pixel_distance)
                    resolution = distance_meters / pixel_dist
                    
                    print(f"✅ 계산된 해상도: {resolution:.4f} meters/pixel")
                    return resolution
            except:
                pass
        
        # 기본값 또는 수동 입력
        print(f"\n일반적인 해상도 값:")
        print(f"  • 0.05 m/pixel: 상세한 실내/캠퍼스 지도")
        print(f"  • 0.1 m/pixel: 일반적인 옥외 지도")
        print(f"  • 0.5 m/pixel: 넓은 지역 지도")
        
        default_resolution = 0.1
        resolution_input = input(f"해상도 입력 (meters/pixel, 기본값: {default_resolution}): ").strip()
        
        try:
            return float(resolution_input) if resolution_input else default_resolution
        except:
            return default_resolution
    
    def create_ros_files(self, processed_img, resolution):
        """ROS 지도 파일들 생성"""
        print(f"\n💾 ROS 지도 파일 생성 중...")
        
        # PGM 파일 저장
        pgm_path = os.path.join(self.map_folder, f"{self.output_name}.pgm")
        cv2.imwrite(pgm_path, processed_img)
        print(f"   ✅ 지도 이미지: {self.output_name}.pgm")
        
        # YAML 파일 생성
        yaml_data = {
            'image': f"./{self.output_name}.pgm",
            'resolution': resolution,
            'origin': [0.0, 0.0, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        
        yaml_path = os.path.join(self.map_folder, f"{self.output_name}.yaml")
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.dump(yaml_data, f, default_flow_style=False)
        print(f"   ✅ 지도 설정: {self.output_name}.yaml")
        
        return pgm_path, yaml_path
    
    def create_separate_images(self, original_img, processed_img):
        """원본 이미지와 ROS 지도 이미지를 별도로 저장"""
        print(f"\n📸 개별 이미지 파일 생성 중...")
        
        try:
            # 1. 원본 이미지를 PNG로 저장 (최적화된 크기)
            original_png_path = os.path.join(self.map_folder, f"{self.output_name}_original.png")
            
            if len(original_img.shape) == 3:
                # BGR to RGB 변환 후 저장
                original_rgb = cv2.cvtColor(original_img, cv2.COLOR_BGR2RGB)
                original_pil = Image.fromarray(original_rgb)
            else:
                original_pil = Image.fromarray(original_img)
            
            # 고품질로 저장
            original_pil.save(original_png_path, 'PNG', optimize=True)
            print(f"   ✅ 원본 지도 이미지: {self.output_name}_original.png")
            
            # 2. ROS 지도를 PNG로도 저장 (웹에서 사용하기 위해)
            ros_png_path = os.path.join(self.map_folder, f"{self.output_name}_ros.png")
            cv2.imwrite(ros_png_path, processed_img)
            print(f"   ✅ ROS 지도 이미지: {self.output_name}_ros.png")
            
            # 3. 비교 미리보기도 생성 (선택사항)
            self.create_comparison_preview(original_img, processed_img)
            
            return original_png_path, ros_png_path
            
        except Exception as e:
            print(f"   ⚠️  이미지 생성 실패: {str(e)}")
            return None, None
    
    def create_comparison_preview(self, original_img, processed_img):
        """원본과 처리된 이미지 비교 미리보기 생성 (참고용)"""
        print(f"   � 비교 미리보기 생성 중...")
        
        try:
            # 크기 맞추기 (미리보기용으로 축소)
            preview_height = 600
            
            # 원본 이미지 리사이즈
            orig_scale = preview_height / original_img.shape[0]
            orig_width = int(original_img.shape[1] * orig_scale)
            orig_preview = cv2.resize(original_img, (orig_width, preview_height))
            
            # 처리된 이미지 리사이즈
            proc_scale = preview_height / processed_img.shape[0]
            proc_width = int(processed_img.shape[1] * proc_scale)
            proc_preview = cv2.resize(processed_img, (proc_width, preview_height))
            
            # 플롯 생성
            fig, axes = plt.subplots(1, 2, figsize=(12, 6))
            
            # 원본 이미지
            if len(orig_preview.shape) == 3:
                orig_rgb = cv2.cvtColor(orig_preview, cv2.COLOR_BGR2RGB)
            else:
                orig_rgb = orig_preview
            axes[0].imshow(orig_rgb)
            axes[0].set_title('Original Map', fontsize=12)
            axes[0].axis('off')
            
            # 처리된 이미지
            axes[1].imshow(proc_preview, cmap='gray')
            axes[1].set_title('ROS Map (White=Free, Black=Obstacle)', fontsize=12)
            axes[1].axis('off')
            
            plt.tight_layout()
            preview_path = os.path.join(self.map_folder, f"{self.output_name}_comparison.png")
            plt.savefig(preview_path, dpi=150, bbox_inches='tight')
            plt.close()
            
            print(f"   ✅ 비교 미리보기: {self.output_name}_comparison.png")
            
        except Exception as e:
            print(f"   ⚠️  비교 미리보기 생성 실패: {str(e)}")
    
    def process(self):
        """전체 처리 과정 실행"""
        print("🗺️ PNG → ROS 지도 변환 및 최적화 시작!")
        print("=" * 60)
        
        # 1. 이미지 로드
        original_img = self.load_and_analyze_image()
        if original_img is None:
            return False
        
        # 2. 크기 최적화
        optimized_img = self.optimize_image_size(original_img)
        
        # 3. 품질 향상
        enhanced_img = self.enhance_map_quality(optimized_img)
        
        # 4. ROS 형식 변환
        ros_img = self.convert_to_ros_format(enhanced_img)
        
        # 5. 해상도 설정
        resolution = self.calculate_resolution_interactive()
        
        # 6. ROS 파일 생성
        pgm_path, yaml_path = self.create_ros_files(ros_img, resolution)
        
        # 7. 개별 이미지 파일 생성
        original_png_path, ros_png_path = self.create_separate_images(enhanced_img, ros_img)
        
        # 8. 요약 출력
        print("\n" + "=" * 60)
        print("🎉 지도 변환 및 최적화 완료!")
        print(f"📁 생성된 파일들:")
        print(f"   • 원본 지도 이미지 (PNG): {self.output_name}_original.png")
        print(f"   • ROS 지도 이미지 (PNG): {self.output_name}_ros.png")
        print(f"   • ROS 지도 이미지 (PGM): {self.output_name}.pgm")
        print(f"   • 지도 설정 파일: {self.output_name}.yaml")
        print(f"   • 비교 미리보기: {self.output_name}_comparison.png")
        
        print(f"\n📊 최종 지도 정보:")
        print(f"   • 크기: {ros_img.shape[1]} x {ros_img.shape[0]} pixels")
        print(f"   • 해상도: {resolution} meters/pixel")
        print(f"   • 실제 범위: {ros_img.shape[1]*resolution:.1f}m x {ros_img.shape[0]*resolution:.1f}m")
        
        print(f"\n🎯 파일 사용 용도:")
        print(f"   • {self.output_name}_original.png: 웹 대시보드용 원본 지도")
        print(f"   • {self.output_name}_ros.png: 웹에서 보기 쉬운 ROS 지도")
        print(f"   • {self.output_name}.pgm + .yaml: ROS 네비게이션용")
        
        print(f"\n🚀 사용 방법:")
        print(f"   ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={yaml_path}")
        print(f"   또는")
        print(f"   ./scripts/quick_start_gps_system.sh {yaml_path}")
        
        return True

def main():
    import sys
    auto_mode = len(sys.argv) > 1 and sys.argv[1] == '--auto'
    
    converter = PngToRosMap(auto_mode=auto_mode)
    success = converter.process()
    
    if success:
        print("\n✨ 모든 작업이 성공적으로 완료되었습니다!")
    else:
        print("\n❌ 작업 중 오류가 발생했습니다.")

if __name__ == "__main__":
    main()