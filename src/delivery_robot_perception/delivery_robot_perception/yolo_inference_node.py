#!/usr/bin/env python3
"""
YOLOv8 기반 객체 인식 노드
카메라 이미지를 수신하여 YOLOv8 모델로 객체를 탐지하고 결과를 발행하는 노드

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import torch
from ultralytics import YOLO
import os
from ament_index_python.packages import get_package_share_directory


class YOLOInferenceNode(Node):
    """YOLOv8 추론을 수행하는 ROS 2 노드"""
    
    def __init__(self):
        super().__init__('yolo_inference_node')
        
        # 파라미터 선언
        self.declare_parameter('model_path', 'yolov8_best.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/yolo/detections')
        self.declare_parameter('publish_debug_image', True)
        
        # 파라미터 가져오기
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        device = self.get_parameter('device').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.publish_debug_image = self.get_parameter('publish_debug_image').get_parameter_value().bool_value
        
        # CV Bridge 초기화
        self.bridge = CvBridge()
        
        # YOLOv8 모델 로드
        try:
            # 프로젝트 루트에서 모델 파일 찾기
            if os.path.isabs(model_path):
                full_model_path = model_path
            else:
                # 상대 경로인 경우 프로젝트 루트의 models 디렉토리에서 찾기
                project_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(__file__))))
                full_model_path = os.path.join(project_root, 'models', model_path)
            
            if os.path.exists(full_model_path):
                self.model = YOLO(full_model_path)
                self.get_logger().info(f"사용자 정의 모델 로드됨: {full_model_path}")
            else:
                self.get_logger().warn(f"모델 파일을 찾을 수 없습니다: {full_model_path}")
                self.get_logger().info("기본 YOLOv8n 모델을 다운로드합니다...")
                self.model = YOLO('yolov8n.pt')  # 자동 다운로드
            
            # GPU 사용 가능한 경우 모델을 GPU로 이동
            if device == 'cuda' and torch.cuda.is_available():
                self.model.to(device)
                self.get_logger().info(f"YOLOv8 모델을 GPU({device})에서 실행합니다.")
            else:
                self.get_logger().info("YOLOv8 모델을 CPU에서 실행합니다.")
                
        except Exception as e:
            self.get_logger().error(f"YOLOv8 모델 로드 실패: {str(e)}")
            return
        
        # Subscriber: 카메라 이미지 수신
        self.image_subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Publisher: 탐지 결과 발행
        self.detection_publisher = self.create_publisher(
            Detection2DArray,
            output_topic,
            10
        )
        
        # Publisher: 디버그용 annotated 이미지 발행 (선택사항)
        if self.publish_debug_image:
            self.debug_image_publisher = self.create_publisher(
                Image,
                '/yolo/debug_image',
                10
            )
        
        self.get_logger().info("YOLOv8 추론 노드가 시작되었습니다.")
        self.get_logger().info(f"입력 토픽: {input_topic}")
        self.get_logger().info(f"출력 토픽: {output_topic}")
        self.get_logger().info(f"신뢰도 임계값: {self.confidence_threshold}")
        
    def image_callback(self, msg: Image):
        """카메라 이미지를 수신하여 YOLOv8 추론을 수행하는 콜백 함수"""
        try:
            # ROS 이미지를 OpenCV 형식으로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # YOLOv8 추론 수행
            results = self.model(cv_image)
            
            # Detection2DArray 메시지 생성
            detection_array = Detection2DArray()
            detection_array.header = Header()
            detection_array.header.stamp = self.get_clock().now().to_msg()
            detection_array.header.frame_id = msg.header.frame_id
            
            # 각 탐지 결과 처리
            for result in results:
                if result.boxes is not None:
                    for box in result.boxes:
                        # 신뢰도 확인
                        confidence = float(box.conf[0])
                        if confidence < self.confidence_threshold:
                            continue
                        
                        # Detection2D 메시지 생성
                        detection = Detection2D()
                        
                        # 바운딩 박스 정보 설정
                        x1, y1, x2, y2 = box.xyxy[0].tolist()
                        detection.bbox.center.position.x = (x1 + x2) / 2.0
                        detection.bbox.center.position.y = (y1 + y2) / 2.0
                        detection.bbox.size_x = x2 - x1
                        detection.bbox.size_y = y2 - y1
                        
                        # 클래스 정보 설정
                        class_id = int(box.cls[0])
                        class_name = self.model.names[class_id]
                        
                        hypothesis = ObjectHypothesisWithPose()
                        hypothesis.hypothesis.class_id = str(class_id)
                        hypothesis.hypothesis.score = confidence
                        
                        detection.results.append(hypothesis)
                        detection_array.detections.append(detection)
                        
                        self.get_logger().debug(
                            f"탐지됨: {class_name} (신뢰도: {confidence:.2f}, "
                            f"위치: [{x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f}])"
                        )
            
            # 탐지 결과 발행
            self.detection_publisher.publish(detection_array)
            
            # 디버그 이미지 생성 및 발행 (선택사항)
            if self.publish_debug_image:
                annotated_image = results[0].plot()  # 바운딩 박스와 라벨이 그려진 이미지
                debug_img_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
                debug_img_msg.header = msg.header
                self.debug_image_publisher.publish(debug_img_msg)
            
            # 탐지된 객체 수 로그
            num_detections = len(detection_array.detections)
            if num_detections > 0:
                self.get_logger().info(f"{num_detections}개의 객체가 탐지되었습니다.")
                
        except CvBridgeError as e:
            self.get_logger().error(f"이미지 변환 오류: {str(e)}")
        except Exception as e:
            self.get_logger().error(f"YOLOv8 추론 오류: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        yolo_node = YOLOInferenceNode()
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()