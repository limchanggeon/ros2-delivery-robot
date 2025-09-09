#!/usr/bin/env python3
"""
QR 코드 인증 노드 - 보안 인증 및 도어 제어
QR 코드를 스캔하여 인증하고 도어 액추에이터를 제어하는 노드

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from zbar_ros_interfaces.msg import Symbol
from control_msgs.msg import DynamicInterfaceGroupValues, InterfaceValue
from delivery_robot_interfaces.srv import AuthenticateQR
from enum import Enum
import hashlib
import time
from typing import Optional, Dict


class AuthenticationState(Enum):
    """인증 상태 정의"""
    IDLE = "idle"
    WAITING_FOR_SENDER = "waiting_for_sender"
    WAITING_FOR_RECEIVER = "waiting_for_receiver"
    DOOR_OPEN = "door_open"
    AUTHENTICATED = "authenticated"
    FAILED = "failed"


class AuthenticationNode(Node):
    """QR 코드 기반 인증 및 도어 제어 노드"""
    
    def __init__(self):
        super().__init__('authentication_node')
        
        # 파라미터 선언
        self.declare_parameter('door_open_time', 10.0)  # 문 열린 시간(초)
        self.declare_parameter('qr_timeout', 30.0)  # QR 코드 대기 시간(초)
        self.declare_parameter('door_servo_pin', 18)  # GPIO 핀 번호
        self.declare_parameter('door_open_angle', 90)  # 문 열릴 때 서보 각도
        self.declare_parameter('door_close_angle', 0)  # 문 닫힐 때 서보 각도
        self.declare_parameter('enable_server_validation', True)  # 서버 검증 활성화
        self.declare_parameter('server_url', 'http://localhost:8080/api/validate_qr')
        
        # 파라미터 가져오기
        self.door_open_time = self.get_parameter('door_open_time').get_parameter_value().double_value
        self.qr_timeout = self.get_parameter('qr_timeout').get_parameter_value().double_value
        self.door_servo_pin = self.get_parameter('door_servo_pin').get_parameter_value().integer_value
        self.door_open_angle = self.get_parameter('door_open_angle').get_parameter_value().integer_value
        self.door_close_angle = self.get_parameter('door_close_angle').get_parameter_value().integer_value
        self.enable_server_validation = self.get_parameter('enable_server_validation').get_parameter_value().bool_value
        self.server_url = self.get_parameter('server_url').get_parameter_value().string_value
        
        # 현재 상태 및 임무 정보
        self.current_state = AuthenticationState.IDLE
        self.current_mission = None
        self.last_qr_scan_time = 0
        self.door_open_start_time = 0
        
        # QR 코드 구독자
        self.qr_subscription = self.create_subscription(
            Symbol,
            '/zbar/symbol',
            self.qr_callback,
            10
        )
        
        # 임무 상태 구독자 (mission_control_node로부터)
        self.mission_subscription = self.create_subscription(
            String,
            '/mission_state',
            self.mission_state_callback,
            10
        )
        
        # 도어 제어 발행자 (ros2_control GPIO 컨트롤러)
        self.door_publisher = self.create_publisher(
            DynamicInterfaceGroupValues,
            '/gpio_controller/commands',
            10
        )
        
        # 인증 상태 발행자
        self.auth_state_publisher = self.create_publisher(
            String,
            '/auth_state',
            10
        )
        
        # 인증 성공/실패 발행자
        self.auth_result_publisher = self.create_publisher(
            Bool,
            '/auth_result',
            10
        )
        
        # QR 인증 서비스
        self.auth_service = self.create_service(
            AuthenticateQR,
            'authenticate_qr',
            self.authenticate_qr_callback
        )
        
        # 주기적 상태 업데이트 타이머
        self.timer = self.create_timer(0.5, self.timer_callback)
        
        self.get_logger().info("QR 인증 노드가 시작되었습니다.")
        self.get_logger().info(f"문 열린 시간: {self.door_open_time}초")
        self.get_logger().info(f"QR 대기 시간: {self.qr_timeout}초")
        self.get_logger().info(f"서버 검증: {'활성화' if self.enable_server_validation else '비활성화'}")
    
    def qr_callback(self, msg: Symbol):
        """QR 코드 스캔 콜백"""
        qr_data = msg.data
        self.get_logger().info(f"QR 코드 스캔됨: {qr_data}")
        
        # 현재 상태에 따른 처리
        if self.current_state == AuthenticationState.WAITING_FOR_SENDER:
            if self.validate_sender_qr(qr_data):
                self.get_logger().info("발송자 인증 성공")
                self.open_door()
                self.publish_auth_result(True)
                # 물품 적재 후 수신자 대기 상태로 변경
                self.current_state = AuthenticationState.WAITING_FOR_RECEIVER
            else:
                self.get_logger().warn("발송자 인증 실패")
                self.publish_auth_result(False)
                
        elif self.current_state == AuthenticationState.WAITING_FOR_RECEIVER:
            if self.validate_receiver_qr(qr_data):
                self.get_logger().info("수신자 인증 성공")
                self.open_door()
                self.publish_auth_result(True)
                self.current_state = AuthenticationState.AUTHENTICATED
            else:
                self.get_logger().warn("수신자 인증 실패")
                self.publish_auth_result(False)
        
        self.last_qr_scan_time = time.time()
    
    def validate_sender_qr(self, qr_data: str) -> bool:
        """발송자 QR 코드 검증"""
        if not self.current_mission:
            return False
        
        expected_qr = self.current_mission.get('pickup_qr_code', '')
        
        # 로컬 검증
        if qr_data == expected_qr:
            # 서버 검증이 활성화된 경우
            if self.enable_server_validation:
                return self.validate_qr_with_server(qr_data, 'pickup')
            return True
        
        return False
    
    def validate_receiver_qr(self, qr_data: str) -> bool:
        """수신자 QR 코드 검증"""
        if not self.current_mission:
            return False
        
        expected_qr = self.current_mission.get('delivery_qr_code', '')
        
        # 로컬 검증
        if qr_data == expected_qr:
            # 서버 검증이 활성화된 경우
            if self.enable_server_validation:
                return self.validate_qr_with_server(qr_data, 'delivery')
            return True
        
        return False
    
    def validate_qr_with_server(self, qr_data: str, qr_type: str) -> bool:
        """서버를 통한 QR 코드 검증"""
        try:
            import requests
            
            payload = {
                'qr_code': qr_data,
                'qr_type': qr_type,
                'mission_id': self.current_mission.get('mission_id', ''),
                'robot_id': self.get_name(),
                'timestamp': time.time()
            }
            
            # QR 코드 해시 생성 (보안 강화)
            qr_hash = hashlib.sha256(qr_data.encode()).hexdigest()
            payload['qr_hash'] = qr_hash
            
            response = requests.post(
                self.server_url,
                json=payload,
                timeout=5.0  # 5초 타임아웃
            )
            
            if response.status_code == 200:
                result = response.json()
                return result.get('valid', False)
            else:
                self.get_logger().error(f"서버 검증 실패: HTTP {response.status_code}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"서버 검증 중 오류: {str(e)}")
            return False
    
    def open_door(self):
        """도어 열기"""
        self.get_logger().info("도어를 엽니다...")
        
        # ros2_control GPIO 컨트롤러에 명령 전송
        door_command = DynamicInterfaceGroupValues()
        door_command.interface_values = []
        
        # 서보 모터 제어 명령 생성
        servo_command = InterfaceValue()
        servo_command.interface_names = [f"gpio_pin_{self.door_servo_pin}"]
        servo_command.values = [float(self.door_open_angle)]
        door_command.interface_values.append(servo_command)
        
        # 명령 발행
        self.door_publisher.publish(door_command)
        
        # 도어 열림 시간 기록
        self.door_open_start_time = time.time()
        self.current_state = AuthenticationState.DOOR_OPEN
        
        self.get_logger().info(f"도어가 열렸습니다. {self.door_open_time}초 후 자동으로 닫힙니다.")
    
    def close_door(self):
        """도어 닫기"""
        self.get_logger().info("도어를 닫습니다...")
        
        # ros2_control GPIO 컨트롤러에 명령 전송
        door_command = DynamicInterfaceGroupValues()
        door_command.interface_values = []
        
        # 서보 모터 제어 명령 생성
        servo_command = InterfaceValue()
        servo_command.interface_names = [f"gpio_pin_{self.door_servo_pin}"]
        servo_command.values = [float(self.door_close_angle)]
        door_command.interface_values.append(servo_command)
        
        # 명령 발행
        self.door_publisher.publish(door_command)
        
        self.get_logger().info("도어가 닫혔습니다.")
    
    def mission_state_callback(self, msg: String):
        """임무 상태 변경 콜백"""
        mission_state = msg.data
        
        if mission_state == "loading":
            self.current_state = AuthenticationState.WAITING_FOR_SENDER
            self.get_logger().info("발송자 QR 코드를 기다리고 있습니다...")
        elif mission_state == "waiting_for_pickup":
            self.current_state = AuthenticationState.WAITING_FOR_RECEIVER
            self.get_logger().info("수신자 QR 코드를 기다리고 있습니다...")
        elif mission_state == "completed" or mission_state == "failed":
            self.current_state = AuthenticationState.IDLE
            self.current_mission = None
    
    def authenticate_qr_callback(self, request, response):
        """QR 인증 서비스 콜백"""
        qr_code = request.qr_code
        qr_type = request.qr_type  # 'pickup' 또는 'delivery'
        
        # 현재 임무 정보 업데이트
        self.current_mission = {
            'pickup_qr_code': request.pickup_qr_code,
            'delivery_qr_code': request.delivery_qr_code,
            'mission_id': request.mission_id
        }
        
        if qr_type == 'pickup':
            result = self.validate_sender_qr(qr_code)
        elif qr_type == 'delivery':
            result = self.validate_receiver_qr(qr_code)
        else:
            result = False
        
        response.success = result
        response.message = "인증 성공" if result else "인증 실패"
        
        return response
    
    def publish_auth_result(self, success: bool):
        """인증 결과 발행"""
        result_msg = Bool()
        result_msg.data = success
        self.auth_result_publisher.publish(result_msg)
    
    def timer_callback(self):
        """주기적 상태 업데이트"""
        current_time = time.time()
        
        # 상태 발행
        state_msg = String()
        state_msg.data = self.current_state.value
        self.auth_state_publisher.publish(state_msg)
        
        # 도어 열림 시간 체크
        if (self.current_state == AuthenticationState.DOOR_OPEN and 
            current_time - self.door_open_start_time >= self.door_open_time):
            self.close_door()
            
            # 상태 복원
            if hasattr(self, '_prev_state'):
                self.current_state = self._prev_state
            else:
                self.current_state = AuthenticationState.IDLE
        
        # QR 코드 대기 타임아웃 체크
        if (self.current_state in [AuthenticationState.WAITING_FOR_SENDER, 
                                  AuthenticationState.WAITING_FOR_RECEIVER] and
            current_time - self.last_qr_scan_time >= self.qr_timeout):
            self.get_logger().warn("QR 코드 대기 시간 초과")
            self.current_state = AuthenticationState.FAILED


def main(args=None):
    rclpy.init(args=args)
    
    try:
        auth_node = AuthenticationNode()
        rclpy.spin(auth_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()