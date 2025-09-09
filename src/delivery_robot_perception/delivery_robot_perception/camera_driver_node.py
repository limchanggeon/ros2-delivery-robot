#!/usr/bin/env python3
"""
카메라 드라이버 노드
USB 카메라 또는 IP 카메라에서 이미지를 수신하여 ROS 토픽으로 발행하는 노드

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import threading
import time
from camera_info_manager import CameraInfoManager


class CameraDriverNode(Node):
    """카메라 드라이버 노드"""
    
    def __init__(self):
        super().__init__('camera_driver_node')
        
        # 파라미터 선언
        self.declare_parameter('device_id', 0)  # USB 카메라 ID
        self.declare_parameter('camera_name', 'delivery_camera')
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('auto_exposure', True)
        self.declare_parameter('exposure_value', 100)
        self.declare_parameter('brightness', 50)
        self.declare_parameter('contrast', 50)
        self.declare_parameter('saturation', 50)
        self.declare_parameter('use_compressed', False)
        
        # 파라미터 가져오기
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.camera_name = self.get_parameter('camera_name').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.image_width = self.get_parameter('image_width').get_parameter_value().integer_value
        self.image_height = self.get_parameter('image_height').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().integer_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.camera_info_topic = self.get_parameter('camera_info_topic').get_parameter_value().string_value
        self.camera_info_url = self.get_parameter('camera_info_url').get_parameter_value().string_value
        self.auto_exposure = self.get_parameter('auto_exposure').get_parameter_value().bool_value
        self.exposure_value = self.get_parameter('exposure_value').get_parameter_value().integer_value
        self.brightness = self.get_parameter('brightness').get_parameter_value().integer_value
        self.contrast = self.get_parameter('contrast').get_parameter_value().integer_value
        self.saturation = self.get_parameter('saturation').get_parameter_value().integer_value
        self.use_compressed = self.get_parameter('use_compressed').get_parameter_value().bool_value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # Camera Info Manager 초기화
        self.camera_info_manager = CameraInfoManager(self, self.camera_name)
        if self.camera_info_url:
            self.camera_info_manager.setURL(self.camera_info_url)
            self.camera_info_manager.loadCameraInfo()
        
        # Publisher 생성
        self.image_publisher = self.create_publisher(
            Image,
            self.image_topic,
            10
        )
        
        self.camera_info_publisher = self.create_publisher(
            CameraInfo,
            self.camera_info_topic,
            10
        )
        
        # 압축 이미지 Publisher (선택사항)
        if self.use_compressed:
            from sensor_msgs.msg import CompressedImage
            self.compressed_image_publisher = self.create_publisher(
                CompressedImage,
                f"{self.image_topic}/compressed",
                10
            )
        
        # 카메라 초기화
        self.cap = None
        self.capturing = False
        self.capture_thread = None
        
        # 카메라 열기
        self.open_camera()
        
        self.get_logger().info(f"카메라 드라이버 노드가 시작되었습니다.")
        self.get_logger().info(f"디바이스 ID: {self.device_id}")
        self.get_logger().info(f"해상도: {self.image_width}x{self.image_height}")
        self.get_logger().info(f"FPS: {self.fps}")
        self.get_logger().info(f"이미지 토픽: {self.image_topic}")
    
    def open_camera(self):
        """카메라 열기"""
        try:
            # OpenCV VideoCapture 생성
            self.cap = cv2.VideoCapture(self.device_id)
            
            if not self.cap.isOpened():
                self.get_logger().error(f"카메라 {self.device_id}를 열 수 없습니다.")
                return False
            
            # 카메라 설정
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
            self.cap.set(cv2.CAP_PROP_FPS, self.fps)
            
            # 노출 설정
            if self.auto_exposure:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)  # 자동 노출
            else:
                self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 수동 노출
                self.cap.set(cv2.CAP_PROP_EXPOSURE, self.exposure_value)
            
            # 기타 설정
            self.cap.set(cv2.CAP_PROP_BRIGHTNESS, self.brightness)
            self.cap.set(cv2.CAP_PROP_CONTRAST, self.contrast)
            self.cap.set(cv2.CAP_PROP_SATURATION, self.saturation)
            
            # 실제 설정된 값 확인
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            actual_fps = int(self.cap.get(cv2.CAP_PROP_FPS))
            
            self.get_logger().info(f"카메라 설정 완료:")
            self.get_logger().info(f"  실제 해상도: {actual_width}x{actual_height}")
            self.get_logger().info(f"  실제 FPS: {actual_fps}")
            
            # 캡처 시작
            self.start_capture()
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"카메라 초기화 실패: {str(e)}")
            return False
    
    def start_capture(self):
        """캡처 스레드 시작"""
        if self.capturing:
            return
        
        self.capturing = True
        self.capture_thread = threading.Thread(target=self.capture_loop)
        self.capture_thread.daemon = True
        self.capture_thread.start()
        
        self.get_logger().info("카메라 캡처가 시작되었습니다.")
    
    def stop_capture(self):
        """캡처 중지"""
        self.capturing = False
        if self.capture_thread and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)
        
        self.get_logger().info("카메라 캡처가 중지되었습니다.")
    
    def capture_loop(self):
        """메인 캡처 루프"""
        frame_time = 1.0 / self.fps
        
        while self.capturing and rclpy.ok():
            start_time = time.time()
            
            try:
                # 프레임 읽기
                ret, frame = self.cap.read()
                
                if not ret:
                    self.get_logger().warn("프레임을 읽을 수 없습니다.")
                    time.sleep(0.1)
                    continue
                
                # 타임스탬프 생성
                timestamp = self.get_clock().now().to_msg()
                
                # ROS 이미지 메시지로 변환
                try:
                    image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                    image_msg.header.stamp = timestamp
                    image_msg.header.frame_id = self.frame_id
                    
                    # 이미지 발행
                    self.image_publisher.publish(image_msg)
                    
                    # 압축 이미지 발행 (선택사항)
                    if self.use_compressed:
                        self.publish_compressed_image(frame, timestamp)
                    
                except CvBridgeError as e:
                    self.get_logger().error(f"이미지 변환 오류: {str(e)}")
                    continue
                
                # 카메라 정보 발행
                self.publish_camera_info(timestamp)
                
                # FPS 조절
                elapsed_time = time.time() - start_time
                sleep_time = max(0, frame_time - elapsed_time)
                if sleep_time > 0:
                    time.sleep(sleep_time)
                
            except Exception as e:
                self.get_logger().error(f"캡처 루프 오류: {str(e)}")
                time.sleep(0.1)
    
    def publish_compressed_image(self, frame, timestamp):
        """압축 이미지 발행"""
        try:
            from sensor_msgs.msg import CompressedImage
            
            # JPEG 압축
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]
            result, encoded_img = cv2.imencode('.jpg', frame, encode_param)
            
            if result:
                compressed_msg = CompressedImage()
                compressed_msg.header.stamp = timestamp
                compressed_msg.header.frame_id = self.frame_id
                compressed_msg.format = "jpeg"
                compressed_msg.data = encoded_img.tobytes()
                
                self.compressed_image_publisher.publish(compressed_msg)
                
        except Exception as e:
            self.get_logger().warn(f"압축 이미지 발행 실패: {str(e)}")
    
    def publish_camera_info(self, timestamp):
        """카메라 정보 발행"""
        camera_info = self.camera_info_manager.getCameraInfo()
        
        if camera_info is None:
            # 기본 카메라 정보 생성
            camera_info = CameraInfo()
            camera_info.width = self.image_width
            camera_info.height = self.image_height
            camera_info.distortion_model = "plumb_bob"
            
            # 기본 내재 매개변수 (실제 캘리브레이션 필요)
            fx = fy = self.image_width  # 임시값
            cx = self.image_width / 2.0
            cy = self.image_height / 2.0
            
            camera_info.k = [fx, 0.0, cx,
                           0.0, fy, cy,
                           0.0, 0.0, 1.0]
            
            camera_info.p = [fx, 0.0, cx, 0.0,
                           0.0, fy, cy, 0.0,
                           0.0, 0.0, 1.0, 0.0]
            
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # 왜곡 없음
        
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = self.frame_id
        
        self.camera_info_publisher.publish(camera_info)
    
    def __del__(self):
        """소멸자"""
        self.close_camera()
    
    def close_camera(self):
        """카메라 닫기"""
        self.stop_capture()
        
        if self.cap and self.cap.isOpened():
            self.cap.release()
            self.get_logger().info("카메라가 해제되었습니다.")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_node = CameraDriverNode()
        rclpy.spin(camera_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()