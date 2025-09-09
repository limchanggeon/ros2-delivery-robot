#!/usr/bin/env python3
"""
시스템 모니터 노드
전체 시스템의 상태를 모니터링하고 진단 정보를 제공하는 노드

작성자: 배달로봇팀
날짜: 2025-09-09
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import psutil
import time
from typing import Dict, List


class SystemMonitorNode(Node):
    """시스템 모니터링 노드"""
    
    def __init__(self):
        super().__init__('system_monitor_node')
        
        # 파라미터 선언
        self.declare_parameter('monitor_frequency', 1.0)
        self.declare_parameter('check_topics', ['/camera/image_raw', '/scan', '/imu/data'])
        self.declare_parameter('topic_timeout', 5.0)
        self.declare_parameter('cpu_threshold', 80.0)
        self.declare_parameter('memory_threshold', 80.0)
        self.declare_parameter('disk_threshold', 90.0)
        
        # 파라미터 가져오기
        self.monitor_frequency = self.get_parameter('monitor_frequency').get_parameter_value().double_value
        self.check_topics = self.get_parameter('check_topics').get_parameter_value().string_array_value
        self.topic_timeout = self.get_parameter('topic_timeout').get_parameter_value().double_value
        self.cpu_threshold = self.get_parameter('cpu_threshold').get_parameter_value().double_value
        self.memory_threshold = self.get_parameter('memory_threshold').get_parameter_value().double_value
        self.disk_threshold = self.get_parameter('disk_threshold').get_parameter_value().double_value
        
        # 시스템 상태 추적
        self.topic_last_seen: Dict[str, float] = {}
        self.system_status = "OK"
        
        # Publisher
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        self.status_publisher = self.create_publisher(
            String,
            '/system_status',
            10
        )
        
        self.health_publisher = self.create_publisher(
            Bool,
            '/system_health',
            10
        )
        
        # 모니터링할 토픽들에 대한 구독자 생성
        self.topic_subscribers = {}
        for topic in self.check_topics:
            self.create_topic_subscriber(topic)
        
        # 타이머 생성
        self.timer = self.create_timer(
            1.0 / self.monitor_frequency,
            self.monitor_callback
        )
        
        self.get_logger().info("시스템 모니터 노드가 시작되었습니다.")
        self.get_logger().info(f"모니터링 주파수: {self.monitor_frequency} Hz")
        self.get_logger().info(f"모니터링 토픽: {self.check_topics}")
    
    def create_topic_subscriber(self, topic_name: str):
        """토픽 구독자 생성"""
        def topic_callback(msg):
            self.topic_last_seen[topic_name] = time.time()
        
        # 토픽 타입에 따라 적절한 구독자 생성
        # 실제 구현에서는 토픽 타입을 동적으로 확인해야 함
        try:
            from std_msgs.msg import Header
            subscriber = self.create_subscription(
                Header,  # 임시로 Header 사용
                topic_name,
                topic_callback,
                10
            )
            self.topic_subscribers[topic_name] = subscriber
            self.get_logger().info(f"토픽 모니터링 시작: {topic_name}")
        except Exception as e:
            self.get_logger().warn(f"토픽 {topic_name} 구독 실패: {str(e)}")
    
    def monitor_callback(self):
        """주기적 모니터링 콜백"""
        current_time = time.time()
        
        # 진단 배열 생성
        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()
        
        # 시스템 전체 상태
        overall_status = DiagnosticStatus.OK
        overall_message = "시스템 정상"
        
        # 1. CPU 사용률 체크
        cpu_status = self.check_cpu_usage()
        diagnostics.status.append(cpu_status)
        if cpu_status.level > overall_status:
            overall_status = cpu_status.level
            overall_message = "CPU 사용률 높음"
        
        # 2. 메모리 사용률 체크
        memory_status = self.check_memory_usage()
        diagnostics.status.append(memory_status)
        if memory_status.level > overall_status:
            overall_status = memory_status.level
            overall_message = "메모리 사용률 높음"
        
        # 3. 디스크 사용률 체크
        disk_status = self.check_disk_usage()
        diagnostics.status.append(disk_status)
        if disk_status.level > overall_status:
            overall_status = disk_status.level
            overall_message = "디스크 사용률 높음"
        
        # 4. 토픽 상태 체크
        topics_status = self.check_topics_status(current_time)
        diagnostics.status.append(topics_status)
        if topics_status.level > overall_status:
            overall_status = topics_status.level
            overall_message = "토픽 연결 문제"
        
        # 5. 온도 체크 (가능한 경우)
        temp_status = self.check_temperature()
        if temp_status:
            diagnostics.status.append(temp_status)
            if temp_status.level > overall_status:
                overall_status = temp_status.level
                overall_message = "온도 경고"
        
        # 진단 정보 발행
        self.diagnostics_publisher.publish(diagnostics)
        
        # 전체 상태 발행
        status_msg = String()
        if overall_status == DiagnosticStatus.OK:
            status_msg.data = "OK"
        elif overall_status == DiagnosticStatus.WARN:
            status_msg.data = "WARNING"
        else:
            status_msg.data = "ERROR"
        
        self.status_publisher.publish(status_msg)
        
        # 헬스 체크 발행
        health_msg = Bool()
        health_msg.data = (overall_status <= DiagnosticStatus.WARN)
        self.health_publisher.publish(health_msg)
        
        # 상태 변경 시 로그
        if self.system_status != status_msg.data:
            self.system_status = status_msg.data
            self.get_logger().info(f"시스템 상태 변경: {overall_message}")
    
    def check_cpu_usage(self) -> DiagnosticStatus:
        """CPU 사용률 체크"""
        status = DiagnosticStatus()
        status.name = "CPU Usage"
        status.hardware_id = "system"
        
        try:
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            status.values.append(KeyValue(key="usage", value=f"{cpu_percent:.1f}%"))
            status.values.append(KeyValue(key="threshold", value=f"{self.cpu_threshold:.1f}%"))
            
            if cpu_percent > self.cpu_threshold:
                status.level = DiagnosticStatus.WARN
                status.message = f"CPU 사용률이 높습니다: {cpu_percent:.1f}%"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"CPU 사용률 정상: {cpu_percent:.1f}%"
                
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"CPU 정보 읽기 실패: {str(e)}"
        
        return status
    
    def check_memory_usage(self) -> DiagnosticStatus:
        """메모리 사용률 체크"""
        status = DiagnosticStatus()
        status.name = "Memory Usage"
        status.hardware_id = "system"
        
        try:
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            
            status.values.append(KeyValue(key="usage", value=f"{memory_percent:.1f}%"))
            status.values.append(KeyValue(key="used", value=f"{memory.used / 1024**3:.1f} GB"))
            status.values.append(KeyValue(key="total", value=f"{memory.total / 1024**3:.1f} GB"))
            status.values.append(KeyValue(key="threshold", value=f"{self.memory_threshold:.1f}%"))
            
            if memory_percent > self.memory_threshold:
                status.level = DiagnosticStatus.WARN
                status.message = f"메모리 사용률이 높습니다: {memory_percent:.1f}%"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"메모리 사용률 정상: {memory_percent:.1f}%"
                
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"메모리 정보 읽기 실패: {str(e)}"
        
        return status
    
    def check_disk_usage(self) -> DiagnosticStatus:
        """디스크 사용률 체크"""
        status = DiagnosticStatus()
        status.name = "Disk Usage"
        status.hardware_id = "system"
        
        try:
            disk = psutil.disk_usage('/')
            disk_percent = (disk.used / disk.total) * 100
            
            status.values.append(KeyValue(key="usage", value=f"{disk_percent:.1f}%"))
            status.values.append(KeyValue(key="used", value=f"{disk.used / 1024**3:.1f} GB"))
            status.values.append(KeyValue(key="total", value=f"{disk.total / 1024**3:.1f} GB"))
            status.values.append(KeyValue(key="threshold", value=f"{self.disk_threshold:.1f}%"))
            
            if disk_percent > self.disk_threshold:
                status.level = DiagnosticStatus.WARN
                status.message = f"디스크 사용률이 높습니다: {disk_percent:.1f}%"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"디스크 사용률 정상: {disk_percent:.1f}%"
                
        except Exception as e:
            status.level = DiagnosticStatus.ERROR
            status.message = f"디스크 정보 읽기 실패: {str(e)}"
        
        return status
    
    def check_topics_status(self, current_time: float) -> DiagnosticStatus:
        """토픽 상태 체크"""
        status = DiagnosticStatus()
        status.name = "Topics Status"
        status.hardware_id = "communication"
        
        inactive_topics = []
        active_topics = []
        
        for topic in self.check_topics:
            last_seen = self.topic_last_seen.get(topic, 0)
            time_since_last = current_time - last_seen
            
            if time_since_last > self.topic_timeout:
                inactive_topics.append(topic)
            else:
                active_topics.append(topic)
            
            status.values.append(KeyValue(
                key=f"last_seen_{topic.replace('/', '_')}",
                value=f"{time_since_last:.1f}s ago"
            ))
        
        if inactive_topics:
            status.level = DiagnosticStatus.WARN
            status.message = f"비활성 토픽: {', '.join(inactive_topics)}"
        else:
            status.level = DiagnosticStatus.OK
            status.message = f"모든 토픽 활성: {len(active_topics)}개"
        
        return status
    
    def check_temperature(self) -> DiagnosticStatus:
        """온도 체크 (Raspberry Pi 등)"""
        status = DiagnosticStatus()
        status.name = "Temperature"
        status.hardware_id = "thermal"
        
        try:
            # Raspberry Pi 온도 읽기
            with open('/sys/class/thermal/thermal_zone0/temp', 'r') as f:
                temp_str = f.read().strip()
                temp_celsius = int(temp_str) / 1000.0
            
            status.values.append(KeyValue(key="temperature", value=f"{temp_celsius:.1f}°C"))
            
            if temp_celsius > 80.0:
                status.level = DiagnosticStatus.ERROR
                status.message = f"온도가 위험 수준입니다: {temp_celsius:.1f}°C"
            elif temp_celsius > 70.0:
                status.level = DiagnosticStatus.WARN
                status.message = f"온도가 높습니다: {temp_celsius:.1f}°C"
            else:
                status.level = DiagnosticStatus.OK
                status.message = f"온도 정상: {temp_celsius:.1f}°C"
            
            return status
            
        except:
            # 온도 센서가 없거나 읽기 실패시 None 반환
            return None


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor_node = SystemMonitorNode()
        rclpy.spin(monitor_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"노드 실행 중 오류 발생: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()