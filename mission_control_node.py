#!/usr/bin/env python3
"""
임무 제어 노드 - 외부 지도 API 연동 및 Nav2 제어
카카오맵 API를 통해 경로를 생성하고 Nav2를 이용해 자율 주행을 관리하는 노드

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose, FollowWaypoints
from std_msgs.msg import String
from delivery_robot_interfaces.action import StartDelivery  # 사용자 정의 액션
from delivery_robot_interfaces.srv import GetRoute  # 사용자 정의 서비스
import tf2_ros
import tf2_geometry_msgs
import requests
import json
import math
from typing import List, Optional
from enum import Enum


class MissionState(Enum):
    """임무 상태 정의"""
    IDLE = "idle"
    LOADING = "loading" 
    DELIVERING = "delivering"
    WAITING_FOR_PICKUP = "waiting_for_pickup"
    RETURNING_HOME = "returning_home"
    COMPLETED = "completed"
    FAILED = "failed"


class MissionControlNode(Node):
    """배송 임무를 관리하는 메인 제어 노드"""
    
    def __init__(self):
        super().__init__('mission_control_node')
        
        # 파라미터 선언
        self.declare_parameter('kakao_api_key', '')
        self.declare_parameter('home_position_x', 0.0)
        self.declare_parameter('home_position_y', 0.0) 
        self.declare_parameter('home_position_yaw', 0.0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('waypoint_tolerance', 2.0)
        
        # 파라미터 가져오기
        self.kakao_api_key = self.get_parameter('kakao_api_key').get_parameter_value().string_value
        self.home_position_x = self.get_parameter('home_position_x').get_parameter_value().double_value
        self.home_position_y = self.get_parameter('home_position_y').get_parameter_value().double_value
        self.home_position_yaw = self.get_parameter('home_position_yaw').get_parameter_value().double_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        self.waypoint_tolerance = self.get_parameter('waypoint_tolerance').get_parameter_value().double_value
        
        # 현재 임무 상태
        self.current_state = MissionState.IDLE
        self.current_mission = None
        self.home_pose = None
        
        # TF 버퍼 및 리스너
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Nav2 액션 클라이언트
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.follow_waypoints_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
        # 배송 시작 액션 서버
        self.start_delivery_server = ActionServer(
            self,
            StartDelivery,
            'start_delivery',
            self.start_delivery_callback
        )
        
        # 경로 생성 서비스 클라이언트
        self.route_service_client = self.create_client(GetRoute, 'get_route_from_api')
        
        # 상태 발행자
        self.state_publisher = self.create_publisher(String, 'mission_state', 10)
        
        # 주기적 상태 체크를 위한 타이머
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.get_logger().info("임무 제어 노드가 시작되었습니다.")
        
        # 홈 위치 저장
        self.save_home_position()
        
    def save_home_position(self):
        """현재 위치를 홈 위치로 저장"""
        try:
            # map에서 base_link로의 변환 가져오기
            transform = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame, 
                rclpy.time.Time()
            )
            
            self.home_pose = PoseStamped()
            self.home_pose.header.frame_id = self.map_frame
            self.home_pose.header.stamp = self.get_clock().now().to_msg()
            self.home_pose.pose.position.x = transform.transform.translation.x
            self.home_pose.pose.position.y = transform.transform.translation.y
            self.home_pose.pose.position.z = 0.0
            self.home_pose.pose.orientation = transform.transform.rotation
            
            self.get_logger().info(
                f"홈 위치 저장됨: ({self.home_pose.pose.position.x:.2f}, "
                f"{self.home_pose.pose.position.y:.2f})"
            )
            
        except Exception as e:
            self.get_logger().warn(f"홈 위치 저장 실패, 설정값 사용: {str(e)}")
            
            # 파라미터에서 설정된 값 사용
            self.home_pose = PoseStamped()
            self.home_pose.header.frame_id = self.map_frame
            self.home_pose.header.stamp = self.get_clock().now().to_msg()
            self.home_pose.pose.position.x = self.home_position_x
            self.home_pose.pose.position.y = self.home_position_y
            self.home_pose.pose.position.z = 0.0
            
            # yaw를 quaternion으로 변환
            qz = math.sin(self.home_position_yaw / 2.0)
            qw = math.cos(self.home_position_yaw / 2.0)
            self.home_pose.pose.orientation.z = qz
            self.home_pose.pose.orientation.w = qw
    
    def start_delivery_callback(self, goal_handle):
        """배송 시작 액션 콜백"""
        request = goal_handle.request
        self.get_logger().info(f"새로운 배송 요청: {request.destination_address}")
        
        # 현재 상태 확인
        if self.current_state != MissionState.IDLE:
            goal_handle.abort()
            result = StartDelivery.Result()
            result.success = False
            result.message = f"현재 상태({self.current_state.value})에서는 새 임무를 시작할 수 없습니다."
            return result
        
        try:
            # 경로 생성 서비스 호출
            route_request = GetRoute.Request()
            route_request.destination_address = request.destination_address
            route_future = self.route_service_client.call_async(route_request)
            
            # 서비스 응답 대기 (비동기)
            rclpy.spin_until_future_complete(self, route_future, timeout_sec=10.0)
            
            if route_future.result() is not None:
                route_response = route_future.result()
                if route_response.success:
                    # 경로점으로 변환
                    waypoints = self.convert_to_poses(route_response.waypoints)
                    
                    # 임무 시작
                    self.current_mission = {
                        'waypoints': waypoints,
                        'destination_address': request.destination_address,
                        'pickup_qr_code': request.pickup_qr_code,
                        'delivery_qr_code': request.delivery_qr_code
                    }
                    
                    # 상태 변경
                    self.current_state = MissionState.LOADING
                    goal_handle.succeed()
                    
                    result = StartDelivery.Result()
                    result.success = True
                    result.message = "배송 임무가 성공적으로 시작되었습니다."
                    
                    # 실제 배송 시작
                    self.execute_delivery()
                    
                    return result
                else:
                    goal_handle.abort()
                    result = StartDelivery.Result()
                    result.success = False
                    result.message = f"경로 생성 실패: {route_response.message}"
                    return result
            else:
                goal_handle.abort()
                result = StartDelivery.Result()
                result.success = False
                result.message = "경로 생성 서비스 호출 시간 초과"
                return result
                
        except Exception as e:
            self.get_logger().error(f"배송 시작 중 오류: {str(e)}")
            goal_handle.abort()
            result = StartDelivery.Result()
            result.success = False
            result.message = f"배송 시작 실패: {str(e)}"
            return result
    
    def convert_to_poses(self, waypoints: List) -> List[PoseStamped]:
        """GPS 좌표 목록을 PoseStamped 목록으로 변환"""
        poses = []
        
        for waypoint in waypoints:
            # GPS 좌표를 map 프레임의 좌표로 변환하는 로직
            # 실제 구현에서는 navsat_transform_node의 결과를 사용해야 함
            pose = PoseStamped()
            pose.header.frame_id = self.map_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # TODO: GPS 좌표를 실제 map 좌표로 변환
            # 여기서는 예시로 단순 변환 사용
            pose.pose.position.x = waypoint.x  # 실제로는 변환 필요
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            
            poses.append(pose)
        
        return poses
    
    def execute_delivery(self):
        """배송 실행"""
        if not self.current_mission:
            return
        
        self.get_logger().info("배송 경로 추종을 시작합니다...")
        self.current_state = MissionState.DELIVERING
        
        # Nav2 FollowWaypoints 액션 호출
        goal_msg = FollowWaypoints.Goal()
        goal_msg.poses = self.current_mission['waypoints']
        
        # 액션 서버가 준비될 때까지 대기
        self.follow_waypoints_client.wait_for_server()
        
        # 액션 전송
        send_goal_future = self.follow_waypoints_client.send_goal_async(
            goal_msg, 
            feedback_callback=self.navigation_feedback_callback
        )
        send_goal_future.add_done_callback(self.navigation_goal_response_callback)
    
    def navigation_goal_response_callback(self, future):
        """내비게이션 액션 응답 콜백"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("내비게이션 목표가 거부되었습니다.")
            self.current_state = MissionState.FAILED
            return
        
        self.get_logger().info("내비게이션 목표가 수락되었습니다.")
        
        # 결과 대기
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.navigation_result_callback)
    
    def navigation_feedback_callback(self, feedback_msg):
        """내비게이션 피드백 콜백"""
        feedback = feedback_msg.feedback
        current_waypoint = feedback.current_waypoint
        self.get_logger().info(f"현재 경유지: {current_waypoint}")
    
    def navigation_result_callback(self, future):
        """내비게이션 결과 콜백"""
        result = future.result().result
        
        if result.result == FollowWaypoints.Result.SUCCESS:
            self.get_logger().info("목적지에 도착했습니다.")
            self.current_state = MissionState.WAITING_FOR_PICKUP
        else:
            self.get_logger().error("내비게이션 실패")
            self.current_state = MissionState.FAILED
            self.return_home()
    
    def return_home(self):
        """홈 위치로 복귀"""
        if not self.home_pose:
            self.get_logger().error("홈 위치가 설정되지 않았습니다.")
            return
        
        self.get_logger().info("홈으로 복귀합니다...")
        self.current_state = MissionState.RETURNING_HOME
        
        # Nav2 NavigateToPose 액션 호출
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.home_pose
        
        self.nav_to_pose_client.wait_for_server()
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.return_home_response_callback)
    
    def return_home_response_callback(self, future):
        """홈 복귀 액션 응답 콜백"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error("홈 복귀 목표가 거부되었습니다.")
            return
        
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.return_home_result_callback)
    
    def return_home_result_callback(self, future):
        """홈 복귀 결과 콜백"""
        result = future.result().result
        
        if result.result == NavigateToPose.Result.SUCCESS:
            self.get_logger().info("홈에 성공적으로 복귀했습니다.")
            self.current_state = MissionState.COMPLETED
            self.current_mission = None
        else:
            self.get_logger().error("홈 복귀 실패")
            self.current_state = MissionState.FAILED
    
    def timer_callback(self):
        """주기적 상태 업데이트"""
        # 현재 상태 발행
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_publisher.publish(state_msg)
        
        # 상태별 처리 로직
        if self.current_state == MissionState.WAITING_FOR_PICKUP:
            # QR 코드 스캔 대기 또는 타임아웃 처리
            pass
        elif self.current_state == MissionState.COMPLETED:
            # 완료 후 IDLE 상태로 복귀
            self.current_state = MissionState.IDLE


def main(args=None):
    rclpy.init(args=args)
    
    try:
        mission_control_node = MissionControlNode()
        rclpy.spin(mission_control_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()