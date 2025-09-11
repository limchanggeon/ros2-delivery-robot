#!/usr/bin/env python3
"""
Enhanced WebBridgeNode for ROS2 Delivery Robot Fleet Management
Bridges ROS2 ecosystem with web-based control system via WebSocket connections.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from lifecycle_msgs.srv import ChangeState, GetState

import asyncio
import websockets
import json
import threading
import time
import ssl
from datetime import datetime
from typing import Dict, Any, Optional

class WebBridgeNode(Node):
    def __init__(self):
        super().__init__('web_bridge_node')
        
        # 로봇 ID 설정
        self.robot_id = self.get_namespace().strip('/') or 'robot_default'
        
        # 백엔드 서버 설정
        self.backend_host = self.declare_parameter('backend_host', 'localhost').value
        self.backend_port = self.declare_parameter('backend_port', 8000).value
        self.use_ssl = self.declare_parameter('use_ssl', False).value
        
        protocol = 'wss' if self.use_ssl else 'ws'
        self.backend_uri = f"{protocol}://{self.backend_host}:{self.backend_port}/ws/robot/{self.robot_id}"
        
        # WebSocket 연결 관리
        self.websocket = None
        self.is_connected = False
        self.reconnect_interval = 5.0
        
        # ROS2 구독자들
        self.setup_subscribers()
        
        # ROS2 발행자들
        self.setup_publishers()
        
        # ROS2 액션 클라이언트들
        self.setup_action_clients()
        
        # ROS2 서비스 클라이언트들
        self.setup_service_clients()
        
        # 메시지 큐 (ROS -> Web)
        self.message_queue = asyncio.Queue()
        
        # 현재 실행 중인 액션들 추적
        self.active_actions = {}
        
        # 미션 실행 엔진
        self.mission_executor = MissionExecutor(self)
        
        # WebSocket 연결 시작 (별도 스레드에서)
        self.websocket_thread = threading.Thread(target=self.start_websocket_connection, daemon=True)
        self.websocket_thread.start()
        
        self.get_logger().info(f'WebBridgeNode initialized for robot: {self.robot_id}')
        self.get_logger().info(f'Backend URI: {self.backend_uri}')

    def setup_subscribers(self):
        """ROS2 구독자 설정"""
        # 로봇 상태 정보
        self.create_subscription(String, 'robot_status', self.status_callback, 10)
        
        # 내비게이션 정보
        self.create_subscription(OccupancyGrid, 'map', self.map_callback, 1)
        self.create_subscription(Path, 'plan', self.path_callback, 10)
        
        # 센서 데이터
        self.create_subscription(String, 'mission_logs', self.mission_log_callback, 10)

    def setup_publishers(self):
        """ROS2 발행자 설정"""
        # 제어 명령
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.goal_pose_publisher = self.create_publisher(PoseStamped, 'goal_pose', 10)
        
        # 미션 관련
        self.mission_status_publisher = self.create_publisher(String, 'mission_status', 10)
        self.mission_log_publisher = self.create_publisher(String, 'mission_logs', 10)

    def setup_action_clients(self):
        """ROS2 액션 클라이언트 설정"""
        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def setup_service_clients(self):
        """ROS2 서비스 클라이언트 설정"""
        # 라이프사이클 관리
        self.change_state_client = self.create_client(ChangeState, 'navigation/change_state')
        self.get_state_client = self.create_client(GetState, 'navigation/get_state')

    # ROS2 콜백 함수들
    def status_callback(self, msg):
        """로봇 상태 콜백 - 백엔드로 전송"""
        try:
            asyncio.run_coroutine_threadsafe(
                self.send_to_backend(msg.data), 
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'Error in status callback: {e}')

    def map_callback(self, msg):
        """맵 데이터 콜백"""
        # 맵 데이터는 크므로 필요시에만 전송
        map_info = {
            'type': 'map_update',
            'robot_id': self.robot_id,
            'width': msg.info.width,
            'height': msg.info.height,
            'resolution': msg.info.resolution,
            'origin': {
                'x': msg.info.origin.position.x,
                'y': msg.info.origin.position.y,
                'theta': msg.info.origin.orientation.z
            }
        }
        
        try:
            asyncio.run_coroutine_threadsafe(
                self.send_to_backend(json.dumps(map_info)), 
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'Error in map callback: {e}')

    def path_callback(self, msg):
        """경로 계획 콜백"""
        if not msg.poses:
            return
            
        path_data = {
            'type': 'path_update',
            'robot_id': self.robot_id,
            'path': [
                {
                    'x': pose.pose.position.x,
                    'y': pose.pose.position.y,
                    'theta': pose.pose.orientation.z
                }
                for pose in msg.poses[::5]  # 5개씩 건너뛰어 데이터 크기 줄이기
            ],
            'timestamp': time.time()
        }
        
        try:
            asyncio.run_coroutine_threadsafe(
                self.send_to_backend(json.dumps(path_data)), 
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'Error in path callback: {e}')

    def mission_log_callback(self, msg):
        """미션 로그 콜백"""
        try:
            log_data = json.loads(msg.data)
            log_data['robot_id'] = self.robot_id
            
            asyncio.run_coroutine_threadsafe(
                self.send_to_backend(json.dumps(log_data)), 
                self.loop
            )
        except Exception as e:
            self.get_logger().error(f'Error in mission log callback: {e}')

    # WebSocket 관련 메서드들
    def start_websocket_connection(self):
        """WebSocket 연결 시작 (별도 스레드에서 실행)"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        
        try:
            self.loop.run_until_complete(self.websocket_handler())
        except Exception as e:
            self.get_logger().error(f'WebSocket handler error: {e}')
        finally:
            self.loop.close()

    async def websocket_handler(self):
        """WebSocket 연결 및 메시지 처리"""
        while True:
            try:
                # SSL 컨텍스트 설정 (필요시)
                ssl_context = None
                if self.use_ssl:
                    ssl_context = ssl.create_default_context()
                    ssl_context.check_hostname = False
                    ssl_context.verify_mode = ssl.CERT_NONE
                
                self.get_logger().info(f'Connecting to {self.backend_uri}...')
                
                async with websockets.connect(
                    self.backend_uri,
                    ssl=ssl_context,
                    ping_interval=30,
                    ping_timeout=10,
                    close_timeout=10
                ) as websocket:
                    self.websocket = websocket
                    self.is_connected = True
                    self.get_logger().info('WebSocket connected successfully')
                    
                    # 연결 확인 메시지 전송
                    await self.send_connection_info()
                    
                    # 송신과 수신을 동시에 처리
                    await asyncio.gather(
                        self.message_receiver(),
                        self.message_sender(),
                        return_exceptions=True
                    )
                    
            except websockets.exceptions.ConnectionClosedError as e:
                self.get_logger().warn(f'WebSocket connection closed: {e}')
            except websockets.exceptions.InvalidStatusCode as e:
                self.get_logger().error(f'WebSocket connection failed with status {e.status_code}')
            except Exception as e:
                self.get_logger().error(f'WebSocket connection error: {e}')
            finally:
                self.is_connected = False
                self.websocket = None
                
                self.get_logger().warn(f'Reconnecting in {self.reconnect_interval} seconds...')
                await asyncio.sleep(self.reconnect_interval)

    async def send_connection_info(self):
        """연결 정보 전송"""
        connection_info = {
            'type': 'robot_connected',
            'robot_id': self.robot_id,
            'timestamp': datetime.now().isoformat(),
            'capabilities': [
                'navigation',
                'telemetry',
                'mission_execution',
                'remote_control'
            ]
        }
        await self.send_to_backend(json.dumps(connection_info))

    async def message_receiver(self):
        """백엔드로부터 메시지 수신"""
        try:
            async for message in self.websocket:
                await self.handle_backend_message(message)
        except websockets.exceptions.ConnectionClosedError:
            pass

    async def message_sender(self):
        """백엔드로 메시지 전송 (큐에서 가져와서)"""
        try:
            while self.is_connected:
                try:
                    # 0.1초 타임아웃으로 큐에서 메시지 가져오기
                    message = await asyncio.wait_for(self.message_queue.get(), timeout=0.1)
                    if self.websocket and self.is_connected:
                        await self.websocket.send(message)
                except asyncio.TimeoutError:
                    continue
                except Exception as e:
                    self.get_logger().error(f'Error sending message: {e}')
                    break
        except Exception as e:
            self.get_logger().error(f'Message sender error: {e}')

    async def send_to_backend(self, message: str):
        """백엔드로 메시지 전송 (큐에 추가)"""
        if hasattr(self, 'message_queue'):
            try:
                self.message_queue.put_nowait(message)
            except asyncio.QueueFull:
                self.get_logger().warn('Message queue is full, dropping message')

    async def handle_backend_message(self, message: str):
        """백엔드로부터 받은 메시지 처리"""
        try:
            data = json.loads(message)
            command_type = data.get('type')
            
            self.get_logger().info(f'Received command: {command_type}')
            
            if command_type == 'navigate_to_pose':
                await self.handle_navigation_command(data.get('payload', {}))
            elif command_type == 'cmd_vel':
                await self.handle_velocity_command(data.get('payload', {}))
            elif command_type == 'execute_mission':
                await self.handle_mission_command(data.get('payload', {}))
            elif command_type == 'emergency_stop':
                await self.handle_emergency_stop()
            elif command_type == 'pause_mission':
                await self.handle_pause_mission()
            elif command_type == 'resume_mission':
                await self.handle_resume_mission()
            elif command_type == 'cancel_mission':
                await self.handle_cancel_mission()
            elif command_type == 'get_status':
                await self.handle_status_request()
            else:
                self.get_logger().warn(f'Unknown command type: {command_type}')
                
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON from backend: {e}')
        except Exception as e:
            self.get_logger().error(f'Error handling backend message: {e}')

    # 명령 처리 메서드들
    async def handle_navigation_command(self, payload: Dict[str, Any]):
        """내비게이션 명령 처리"""
        try:
            x = payload.get('x', 0.0)
            y = payload.get('y', 0.0)
            z = payload.get('z', 0.0)
            orientation_w = payload.get('orientation_w', 1.0)
            
            # NavigateToPose 액션 목표 생성
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = float(x)
            goal_msg.pose.pose.position.y = float(y)
            goal_msg.pose.pose.position.z = float(z)
            goal_msg.pose.pose.orientation.w = float(orientation_w)
            
            # 액션 서버가 준비될 때까지 대기
            if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
                raise Exception('Navigation action server not available')
            
            # 액션 전송
            future = self.nav_action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.navigation_feedback_callback
            )
            
            # 결과 처리를 위한 콜백 등록
            future.add_done_callback(
                lambda f: self.navigation_result_callback(f, x, y)
            )
            
            # 로그 발행
            self.publish_mission_log(
                f'Navigation goal sent: ({x:.2f}, {y:.2f})', 
                'info'
            )
            
        except Exception as e:
            self.get_logger().error(f'Navigation command failed: {e}')
            self.publish_mission_log(f'Navigation command failed: {e}', 'error')

    def navigation_feedback_callback(self, feedback_msg):
        """내비게이션 피드백 콜백"""
        feedback = feedback_msg.feedback
        # 진행 상황을 백엔드로 전송
        progress_data = {
            'type': 'navigation_progress',
            'robot_id': self.robot_id,
            'distance_remaining': feedback.distance_remaining,
            'estimated_time_remaining': feedback.estimated_time_remaining.sec,
            'timestamp': time.time()
        }
        
        asyncio.run_coroutine_threadsafe(
            self.send_to_backend(json.dumps(progress_data)), 
            self.loop
        )

    def navigation_result_callback(self, future, target_x, target_y):
        """내비게이션 결과 콜백"""
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.publish_mission_log('Navigation goal rejected', 'error')
                return
            
            # 결과 대기
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f: self.handle_navigation_result(f, target_x, target_y)
            )
            
        except Exception as e:
            self.get_logger().error(f'Navigation result callback error: {e}')

    def handle_navigation_result(self, future, target_x, target_y):
        """내비게이션 결과 처리"""
        try:
            result = future.result()
            if result.status == 4:  # SUCCEEDED
                self.publish_mission_log(
                    f'Navigation successful to ({target_x:.2f}, {target_y:.2f})', 
                    'info'
                )
            else:
                self.publish_mission_log(
                    f'Navigation failed with status: {result.status}', 
                    'error'
                )
        except Exception as e:
            self.get_logger().error(f'Navigation result handling error: {e}')

    async def handle_velocity_command(self, payload: Dict[str, Any]):
        """속도 명령 처리"""
        try:
            twist_msg = Twist()
            
            linear = payload.get('linear', {})
            angular = payload.get('angular', {})
            
            twist_msg.linear.x = float(linear.get('x', 0.0))
            twist_msg.linear.y = float(linear.get('y', 0.0))
            twist_msg.linear.z = float(linear.get('z', 0.0))
            
            twist_msg.angular.x = float(angular.get('x', 0.0))
            twist_msg.angular.y = float(angular.get('y', 0.0))
            twist_msg.angular.z = float(angular.get('z', 0.0))
            
            self.cmd_vel_publisher.publish(twist_msg)
            
        except Exception as e:
            self.get_logger().error(f'Velocity command failed: {e}')

    async def handle_mission_command(self, payload: Dict[str, Any]):
        """미션 실행 명령 처리"""
        try:
            mission_id = payload.get('mission_id', 'unknown')
            definition = payload.get('definition', {})
            
            # 미션 실행기에 미션 전달
            await self.mission_executor.execute_mission(mission_id, definition)
            
        except Exception as e:
            self.get_logger().error(f'Mission command failed: {e}')
            self.publish_mission_log(f'Mission command failed: {e}', 'error')

    async def handle_emergency_stop(self):
        """비상 정지 처리"""
        try:
            # 모든 액션 취소
            for action_future in self.active_actions.values():
                if not action_future.done():
                    action_future.cancel()
            
            # 속도를 0으로 설정
            stop_msg = Twist()
            self.cmd_vel_publisher.publish(stop_msg)
            
            # 미션 중단
            await self.mission_executor.stop_mission()
            
            self.publish_mission_log('Emergency stop activated', 'warn')
            
        except Exception as e:
            self.get_logger().error(f'Emergency stop failed: {e}')

    async def handle_pause_mission(self):
        """미션 일시정지"""
        await self.mission_executor.pause_mission()
        self.publish_mission_log('Mission paused', 'info')

    async def handle_resume_mission(self):
        """미션 재개"""
        await self.mission_executor.resume_mission()
        self.publish_mission_log('Mission resumed', 'info')

    async def handle_cancel_mission(self):
        """미션 취소"""
        await self.mission_executor.cancel_mission()
        self.publish_mission_log('Mission cancelled', 'info')

    async def handle_status_request(self):
        """상태 요청 처리"""
        status_data = {
            'type': 'status_response',
            'robot_id': self.robot_id,
            'is_connected': self.is_connected,
            'active_actions': len(self.active_actions),
            'mission_status': self.mission_executor.get_status(),
            'timestamp': time.time()
        }
        
        await self.send_to_backend(json.dumps(status_data))

    def publish_mission_log(self, message: str, level: str = 'info'):
        """미션 로그 발행"""
        log_data = {
            'type': 'mission_log',
            'robot_id': self.robot_id,
            'level': level,
            'message': message,
            'timestamp': datetime.now().isoformat()
        }
        
        msg = String()
        msg.data = json.dumps(log_data)
        self.mission_log_publisher.publish(msg)


class MissionExecutor:
    """미션 실행 엔진"""
    
    def __init__(self, bridge_node: WebBridgeNode):
        self.bridge = bridge_node
        self.current_mission = None
        self.mission_status = 'idle'  # idle, running, paused, completed, failed
        self.current_step = 0
        self.mission_future = None
        
    async def execute_mission(self, mission_id: str, definition: Dict[str, Any]):
        """미션 실행"""
        try:
            self.current_mission = {
                'id': mission_id,
                'definition': definition,
                'start_time': time.time()
            }
            self.mission_status = 'running'
            self.current_step = 0
            
            nodes = definition.get('nodes', [])
            edges = definition.get('edges', [])
            
            # 노드 실행 순서 결정 (간단한 순차 실행)
            execution_order = self._determine_execution_order(nodes, edges)
            
            for i, node in enumerate(execution_order):
                if self.mission_status != 'running':
                    break
                    
                self.current_step = i
                await self._execute_node(node)
                
            if self.mission_status == 'running':
                self.mission_status = 'completed'
                self.bridge.publish_mission_log('Mission completed successfully', 'info')
            
        except Exception as e:
            self.mission_status = 'failed'
            self.bridge.publish_mission_log(f'Mission failed: {e}', 'error')

    def _determine_execution_order(self, nodes: list, edges: list) -> list:
        """노드 실행 순서 결정 (토폴로지 정렬 간소화)"""
        # 간단한 구현: 노드 ID 순으로 정렬
        return sorted(nodes, key=lambda x: x.get('id', ''))

    async def _execute_node(self, node: Dict[str, Any]):
        """개별 노드 실행"""
        node_type = node.get('type')
        node_id = node.get('id', 'unknown')
        
        self.bridge.publish_mission_log(f'Executing node {node_id}: {node_type}', 'info')
        
        if node_type == 'navigate_to':
            await self._execute_navigation_node(node)
        elif node_type == 'wait':
            await self._execute_wait_node(node)
        elif node_type == 'action':
            await self._execute_action_node(node)
        else:
            self.bridge.get_logger().warn(f'Unknown node type: {node_type}')

    async def _execute_navigation_node(self, node: Dict[str, Any]):
        """내비게이션 노드 실행"""
        params = node.get('params', {})
        x = params.get('x', 0.0)
        y = params.get('y', 0.0)
        
        # 내비게이션 명령 실행 (동기적으로 대기)
        await self.bridge.handle_navigation_command(params)
        
        # 내비게이션 완료까지 대기 (간소화)
        await asyncio.sleep(1.0)  # 실제로는 액션 결과를 기다려야 함

    async def _execute_wait_node(self, node: Dict[str, Any]):
        """대기 노드 실행"""
        params = node.get('params', {})
        duration = params.get('duration', 1.0)
        
        await asyncio.sleep(duration)

    async def _execute_action_node(self, node: Dict[str, Any]):
        """액션 노드 실행"""
        params = node.get('params', {})
        action_type = params.get('action_type', 'unknown')
        
        # 커스텀 액션 실행 로직
        self.bridge.publish_mission_log(f'Executing action: {action_type}', 'info')
        await asyncio.sleep(0.5)  # 가짜 실행 시간

    async def pause_mission(self):
        """미션 일시정지"""
        if self.mission_status == 'running':
            self.mission_status = 'paused'

    async def resume_mission(self):
        """미션 재개"""
        if self.mission_status == 'paused':
            self.mission_status = 'running'

    async def cancel_mission(self):
        """미션 취소"""
        self.mission_status = 'cancelled'
        if self.mission_future and not self.mission_future.done():
            self.mission_future.cancel()

    async def stop_mission(self):
        """미션 중단"""
        await self.cancel_mission()

    def get_status(self) -> Dict[str, Any]:
        """현재 미션 상태 반환"""
        return {
            'status': self.mission_status,
            'current_step': self.current_step,
            'mission_id': self.current_mission['id'] if self.current_mission else None
        }


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = WebBridgeNode()
        
        # 멀티스레드 실행자 사용
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            executor.shutdown()
            
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()