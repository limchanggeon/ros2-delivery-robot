import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
from sensor_msgs.msg import NavSatFix
import tf2_ros
from pyproj import Proj
import math

class GpsMapCalibrator(Node):
    """
    GPS 지도 보정 노드 - RViz에서 클릭한 지점과 현재 GPS 위치를 이용해
    UTM -> MAP 정적 변환을 자동으로 계산하고 발행하는 노드
    
    구독 토픽:
    - /clicked_point: RViz에서 클릭한 지점 좌표 (geometry_msgs/PointStamped)
    - /gps/fix: GPS 수신기로부터의 위치 데이터 (sensor_msgs/NavSatFix)
    - /imu/data: IMU로부터의 방위각 데이터 (sensor_msgs/Imu) - 선택사항
    
    발행:
    - UTM -> MAP 정적 변환 (TF2 Static Transform)
    """
    def __init__(self):
        super().__init__('gps_map_calibrator')
        
        # TF2 관련 초기화
        self.tf_static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # 구독자(Subscriber) 초기화
        self.create_subscription(PointStamped, '/clicked_point', self.clicked_point_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fix', self.gps_callback, 10)
        
        self.get_logger().info('GPS Map Calibrator Node is ready.')
        self.get_logger().info('Please click a point on the map in RViz to set the origin.')
        
        self.latest_gps = None
        self.utm_proj = None

    def gps_callback(self, msg: NavSatFix):
        # GPS 신호 품질 검사
        if msg.status.status < 0:  # GPS_STATUS_NO_FIX
            self.get_logger().warn('GPS 신호 없음 - 수신 대기 중...')
            return
            
        if msg.latitude == 0.0 and msg.longitude == 0.0:
            self.get_logger().warn('유효하지 않은 GPS 좌표 수신')
            return
            
        self.latest_gps = msg
        
        # UTM 존 자동 계산 (첫 GPS 수신 시)
        if self.utm_proj is None and msg.longitude != 0.0:
            utm_zone = int((msg.longitude + 180) / 6) + 1
            # UTM 존이 60을 넘지 않도록 보정 (극지방 고려)
            if utm_zone > 60:
                utm_zone = 60
            
            self.utm_proj = Proj(proj='utm', zone=utm_zone, ellps='WGS84', preserve_units=False)
            self.get_logger().info(f'📍 UTM 존 설정 완료: {utm_zone}')
            self.get_logger().info(f'📡 GPS 수신 품질: {msg.status.status} (0=정상, -1=신호없음)')
            self.get_logger().info('✅ 이제 RViz에서 지도 위 한 점을 클릭하여 보정을 시작하세요!')

    def clicked_point_callback(self, msg: PointStamped):
        if self.latest_gps is None or self.utm_proj is None:
            self.get_logger().warn('⏳ GPS 수신 대기 중 - 유효한 GPS 신호를 받은 후 보정이 가능합니다.')
            return
            
        # 좌표계 검증
        if msg.header.frame_id != 'map':
            self.get_logger().warn(f'⚠️  잘못된 좌표계: {msg.header.frame_id}. RViz의 Fixed Frame을 "map"으로 설정해주세요.')
            return

        try:
            # 1. 현재 GPS 좌표를 UTM 좌표로 변환
            utm_x, utm_y = self.utm_proj(self.latest_gps.longitude, self.latest_gps.latitude)
            
            # 2. 클릭된 map 좌표
            map_x = msg.point.x
            map_y = msg.point.y
            
            self.get_logger().info('🎯 보정 지점 선택됨:')
            self.get_logger().info(f'   📍 지도 좌표: (x={map_x:.3f}, y={map_y:.3f}) [m]')
            self.get_logger().info(f'   🌍 GPS 좌표: (위도={self.latest_gps.latitude:.8f}°, 경도={self.latest_gps.longitude:.8f}°)')
            self.get_logger().info(f'   📐 UTM 좌표: (x={utm_x:.3f}, y={utm_y:.3f}) [m]')
            
        except Exception as e:
            self.get_logger().error(f'❌ UTM 변환 오류: {str(e)}')
            return

            # 3. UTM -> MAP 변환 계산
            # 사용자가 클릭한 지점(map_x, map_y)이 로봇의 현재 위치(utm_x, utm_y)와 일치한다고 가정
            # 따라서 map 좌표계의 원점은 UTM 좌표계에서 (utm_x - map_x, utm_y - map_y)에 위치
            
            translation_x = utm_x - map_x
            translation_y = utm_y - map_y
            
            # 변환 거리 확인 (비합리적으로 큰 변환값 감지)
            transform_distance = math.sqrt(translation_x**2 + translation_y**2)
            if transform_distance > 100000:  # 100km 이상
                self.get_logger().warn(f'⚠️  매우 큰 변환 거리 감지: {transform_distance:.1f}m')
                self.get_logger().warn('   GPS 좌표나 지도 해상도를 다시 확인해주세요.')
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = 'utm'
            t.child_frame_id = 'map'
            
            # 변환(Translation) 설정
            t.transform.translation.x = translation_x
            t.transform.translation.y = translation_y
            t.transform.translation.z = 0.0
            
            # 회전(Rotation)은 기본적으로 0 (map과 utm의 축이 정렬되었다고 가정)
            # 향후 IMU 데이터를 활용한 방위각 보정 가능
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0
            t.transform.rotation.w = 1.0
            
            # 정적 변환 발행
            self.tf_static_broadcaster.sendTransform(t)
            
            self.get_logger().info('✅ UTM -> MAP 정적 변환 발행 완료!')
            self.get_logger().info(f'   📏 변환값: dx={translation_x:.3f}m, dy={translation_y:.3f}m')
            self.get_logger().info('   🎉 지도가 GPS 좌표에 성공적으로 정렬되었습니다!')
            self.get_logger().info('   💡 이제 이 노드를 종료해도 변환이 유지됩니다.')
            
            # 변환 정보를 파일로 저장 (선택사항)
            self.save_calibration_data(translation_x, translation_y, utm_x, utm_y, map_x, map_y)

    def save_calibration_data(self, tx, ty, utm_x, utm_y, map_x, map_y):
        """보정 데이터를 YAML 파일로 저장하여 향후 자동 로딩 가능하도록 함"""
        import yaml
        import os
        from datetime import datetime
        
        try:
            # 보정 데이터 딕셔너리 생성
            calibration_data = {
                'calibration_info': {
                    'timestamp': datetime.now().strftime('%Y-%m-%d %H:%M:%S'),
                    'utm_zone': int((self.latest_gps.longitude + 180) / 6) + 1,
                    'gps_coordinates': {
                        'latitude': float(self.latest_gps.latitude),
                        'longitude': float(self.latest_gps.longitude)
                    },
                    'map_coordinates': {
                        'x': float(map_x),
                        'y': float(map_y)
                    },
                    'utm_coordinates': {
                        'x': float(utm_x),
                        'y': float(utm_y)
                    },
                    'transform': {
                        'translation_x': float(tx),
                        'translation_y': float(ty),
                        'translation_z': 0.0,
                        'rotation_x': 0.0,
                        'rotation_y': 0.0,
                        'rotation_z': 0.0,
                        'rotation_w': 1.0
                    }
                }
            }
            
            # 설정 파일 저장
            config_dir = os.path.expanduser('~/.ros2_gps_calibration')
            os.makedirs(config_dir, exist_ok=True)
            config_file = os.path.join(config_dir, 'map_calibration.yaml')
            
            with open(config_file, 'w', encoding='utf-8') as f:
                yaml.dump(calibration_data, f, default_flow_style=False, allow_unicode=True)
                
            self.get_logger().info(f'💾 보정 데이터 저장됨: {config_file}')
            
        except Exception as e:
            self.get_logger().warn(f'⚠️  보정 데이터 저장 실패: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = GpsMapCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()