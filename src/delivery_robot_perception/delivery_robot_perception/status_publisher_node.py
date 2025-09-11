#!/usr/bin/env python3
"""
Enhanced StatusPublisherNode for ROS2 Delivery Robot Fleet Management
Collects comprehensive telemetry data and publishes it for web dashboard consumption.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import BatteryState, Image, CompressedImage
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from tf2_ros import TransformListener, Buffer
import psutil
import json
import subprocess
import time
import platform
from datetime import datetime

class StatusPublisherNode(Node):
    def __init__(self):
        super().__init__('status_publisher_node')
        
        # 네임스페이스에서 로봇 ID 추출
        self.robot_id = self.get_namespace().strip('/') or 'robot_default'
        
        # 상태 발행자
        self.publisher_ = self.create_publisher(String, 'robot_status', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)  # 1Hz
        
        # 다양한 토픽 구독
        self.setup_subscriptions()
        
        # TF2 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 데이터 저장소
        self.status_data = {
            'robot_id': self.robot_id,
            'timestamp': None,
            'battery': None,
            'pose': None,
            'velocity': None,
            'system': None,
            'wifi': None,
            'diagnostics': None,
            'navigation': {
                'current_goal': None,
                'path_length': 0,
                'distance_to_goal': None
            },
            'mission': {
                'current_mission': None,
                'mission_status': 'idle',
                'progress': 0.0
            }
        }
        
        self.get_logger().info(f'StatusPublisherNode initialized for robot: {self.robot_id}')

    def setup_subscriptions(self):
        """구독할 토픽들 설정"""
        # 기본 센서 데이터
        self.create_subscription(BatteryState, 'battery_state', self.battery_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(DiagnosticArray, 'diagnostics', self.diagnostics_callback, 10)
        
        # 내비게이션 관련
        self.create_subscription(Path, 'plan', self.path_callback, 10)
        self.create_subscription(PoseStamped, 'goal_pose', self.goal_callback, 10)
        
        # 미션 관련 (커스텀 토픽)
        self.create_subscription(String, 'mission_status', self.mission_callback, 10)
        
        self.get_logger().info('All subscriptions set up successfully')

    def battery_callback(self, msg):
        """배터리 상태 콜백"""
        self.status_data['battery'] = {
            'percentage': msg.percentage,
            'voltage': msg.voltage,
            'current': msg.current,
            'charge': msg.charge,
            'capacity': msg.capacity,
            'design_capacity': msg.design_capacity,
            'power_supply_status': msg.power_supply_status,
            'power_supply_health': msg.power_supply_health,
            'power_supply_technology': msg.power_supply_technology,
            'present': msg.present
        }

    def odom_callback(self, msg):
        """오도메트리 콜백"""
        # 위치 정보
        pos = msg.pose.pose.position
        orient = msg.pose.pose.orientation
        
        # 2D 환경에서 yaw 각도 계산 (간단한 근사)
        # 정확한 계산을 위해서는 tf_transformations 사용 권장
        import math
        siny_cosp = 2 * (orient.w * orient.z + orient.x * orient.y)
        cosy_cosp = 1 - 2 * (orient.y * orient.y + orient.z * orient.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        self.status_data['pose'] = {
            'x': pos.x,
            'y': pos.y,
            'z': pos.z,
            'theta': yaw,
            'orientation': {
                'x': orient.x,
                'y': orient.y,
                'z': orient.z,
                'w': orient.w
            }
        }
        
        # 속도 정보
        vel = msg.twist.twist
        self.status_data['velocity'] = {
            'linear': {
                'x': vel.linear.x,
                'y': vel.linear.y,
                'z': vel.linear.z
            },
            'angular': {
                'x': vel.angular.x,
                'y': vel.angular.y,
                'z': vel.angular.z
            }
        }

    def diagnostics_callback(self, msg):
        """진단 정보 콜백"""
        diagnostics = {}
        for status in msg.status:
            diagnostics[status.name] = {
                'level': status.level,
                'message': status.message,
                'hardware_id': status.hardware_id,
                'values': {kv.key: kv.value for kv in status.values}
            }
        self.status_data['diagnostics'] = diagnostics

    def path_callback(self, msg):
        """계획된 경로 콜백"""
        if msg.poses:
            self.status_data['navigation']['path_length'] = len(msg.poses)
            
            # 현재 위치에서 목표까지의 거리 계산
            if self.status_data['pose']:
                current_pos = self.status_data['pose']
                goal_pos = msg.poses[-1].pose.position
                
                import math
                distance = math.sqrt(
                    (goal_pos.x - current_pos['x'])**2 + 
                    (goal_pos.y - current_pos['y'])**2
                )
                self.status_data['navigation']['distance_to_goal'] = distance

    def goal_callback(self, msg):
        """목표 위치 콜백"""
        pos = msg.pose.position
        self.status_data['navigation']['current_goal'] = {
            'x': pos.x,
            'y': pos.y,
            'z': pos.z,
            'timestamp': msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        }

    def mission_callback(self, msg):
        """미션 상태 콜백"""
        try:
            mission_data = json.loads(msg.data)
            self.status_data['mission'].update(mission_data)
        except json.JSONDecodeError:
            self.get_logger().warn(f'Invalid mission status JSON: {msg.data}')

    def get_system_stats(self):
        """시스템 리소스 정보 수집"""
        try:
            # CPU 및 메모리 사용률
            cpu_percent = psutil.cpu_percent(interval=0.1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')
            
            # 네트워크 통계
            net_io = psutil.net_io_counters()
            
            # 온도 정보 (가능한 경우)
            temperatures = {}
            try:
                temps = psutil.sensors_temperatures()
                for name, entries in temps.items():
                    temperatures[name] = [{'label': entry.label, 'current': entry.current} for entry in entries]
            except (AttributeError, OSError):
                pass
            
            return {
                'cpu_percent': cpu_percent,
                'cpu_count': psutil.cpu_count(),
                'memory': {
                    'total': memory.total,
                    'available': memory.available,
                    'percent': memory.percent,
                    'used': memory.used
                },
                'disk': {
                    'total': disk.total,
                    'used': disk.used,
                    'free': disk.free,
                    'percent': (disk.used / disk.total) * 100
                },
                'network': {
                    'bytes_sent': net_io.bytes_sent,
                    'bytes_recv': net_io.bytes_recv,
                    'packets_sent': net_io.packets_sent,
                    'packets_recv': net_io.packets_recv
                },
                'temperatures': temperatures,
                'boot_time': psutil.boot_time(),
                'uptime': time.time() - psutil.boot_time()
            }
        except Exception as e:
            self.get_logger().error(f'Error collecting system stats: {e}')
            return {'error': str(e)}

    def get_wifi_stats(self):
        """Wi-Fi 연결 정보 수집"""
        try:
            wifi_info = {}
            
            # Linux 시스템에서 iwconfig 사용
            if platform.system() == 'Linux':
                try:
                    # iwconfig 명령어로 무선 인터페이스 정보 가져오기
                    result = subprocess.run(['iwconfig'], capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        output = result.stdout
                        lines = output.split('\n')
                        
                        current_interface = None
                        for line in lines:
                            if 'IEEE 802.11' in line:
                                current_interface = line.split()[0]
                                wifi_info[current_interface] = {}
                            elif current_interface and 'Signal level' in line:
                                # Signal level 파싱
                                parts = line.split('Signal level=')
                                if len(parts) > 1:
                                    signal_part = parts[1].split()[0]
                                    try:
                                        signal_dbm = int(signal_part)
                                        wifi_info[current_interface]['signal_dbm'] = signal_dbm
                                        wifi_info[current_interface]['signal_quality'] = max(0, min(100, (signal_dbm + 100) * 2))
                                    except ValueError:
                                        pass
                            elif current_interface and 'ESSID:' in line:
                                essid = line.split('ESSID:')[1].strip().strip('"')
                                wifi_info[current_interface]['essid'] = essid
                                
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
                    pass
                
                # ip route로 기본 게이트웨이 확인
                try:
                    result = subprocess.run(['ip', 'route', 'show', 'default'], capture_output=True, text=True, timeout=5)
                    if result.returncode == 0:
                        wifi_info['default_gateway'] = result.stdout.strip()
                except:
                    pass
            
            # 일반적인 네트워크 인터페이스 정보
            addrs = psutil.net_if_addrs()
            stats = psutil.net_if_stats()
            
            for interface_name, interface_addrs in addrs.items():
                if interface_name.startswith(('wlan', 'wifi', 'wlp')):
                    if interface_name not in wifi_info:
                        wifi_info[interface_name] = {}
                    
                    # IP 주소 정보
                    for addr in interface_addrs:
                        if addr.family == 2:  # AF_INET
                            wifi_info[interface_name]['ip_address'] = addr.address
                            wifi_info[interface_name]['netmask'] = addr.netmask
                    
                    # 인터페이스 통계
                    if interface_name in stats:
                        stat = stats[interface_name]
                        wifi_info[interface_name]['is_up'] = stat.isup
                        wifi_info[interface_name]['speed'] = stat.speed
                        wifi_info[interface_name]['mtu'] = stat.mtu
            
            return wifi_info if wifi_info else {'status': 'no_wifi_detected'}
            
        except Exception as e:
            self.get_logger().error(f'Error collecting WiFi stats: {e}')
            return {'error': str(e)}

    def get_tf_info(self):
        """TF 변환 정보 수집"""
        try:
            tf_info = {}
            
            # map -> base_link 변환 시도
            try:
                transform = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                
                tf_info['map_to_base_link'] = {
                    'translation': {
                        'x': translation.x,
                        'y': translation.y,
                        'z': translation.z
                    },
                    'rotation': {
                        'x': rotation.x,
                        'y': rotation.y,
                        'z': rotation.z,
                        'w': rotation.w
                    },
                    'timestamp': transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
                }
            except Exception as e:
                tf_info['map_to_base_link'] = {'error': str(e)}
            
            return tf_info
            
        except Exception as e:
            self.get_logger().error(f'Error collecting TF info: {e}')
            return {'error': str(e)}

    def timer_callback(self):
        """주기적으로 상태 정보 발행"""
        # 현재 시간 업데이트
        self.status_data['timestamp'] = datetime.now().isoformat()
        
        # 시스템 정보 업데이트
        self.status_data['system'] = self.get_system_stats()
        self.status_data['wifi'] = self.get_wifi_stats()
        self.status_data['tf'] = self.get_tf_info()
        
        # JSON 메시지 생성 및 발행
        try:
            msg = String()
            msg.data = json.dumps(self.status_data, default=str)
            self.publisher_.publish(msg)
            
            # 로그 출력 (간략하게)
            if self.status_data['battery']:
                battery_pct = self.status_data['battery']['percentage']
                cpu_pct = self.status_data['system']['cpu_percent']
                self.get_logger().info(
                    f'Status published - Battery: {battery_pct:.1f}%, CPU: {cpu_pct:.1f}%',
                    throttle_duration_sec=10  # 10초마다 한 번씩만 로그 출력
                )
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StatusPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()