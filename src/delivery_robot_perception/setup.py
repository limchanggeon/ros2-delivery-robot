from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_robot_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 - 명시적 설치
        (os.path.join('share', package_name, 'launch'), [
            'launch/perception.launch.py'
        ]),
        # 설정 파일
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
        # 모델 파일 (있는 경우)
        (os.path.join('share', package_name, 'models'), 
         glob('models/*.pt') + glob('models/*.onnx')),
        # 플러그인 설명 파일
        (os.path.join('share', package_name), 
         glob('plugins.xml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='배달로봇팀',
    maintainer_email='robotics.team@university.ac.kr',
    description='배달 로봇용 인식 시스템 - YOLOv8 객체 탐지, 카메라 드라이버',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # YOLOv8 추론 노드
            'yolo_inference_node = delivery_robot_perception.yolo_inference_node:main',
            # 카메라 드라이버 노드  
            'camera_driver_node = delivery_robot_perception.camera_driver_node:main',
            # NARCHON 통합 관제 시스템 노드들
            'status_publisher_node = delivery_robot_perception.status_publisher_node:main',
            'web_bridge_node = delivery_robot_perception.web_bridge_node:main',
        ],
    },
    classifiers=[
        'Development Status :: 4 - Beta',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Scientific/Engineering :: Image Recognition',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    python_requires='>=3.8',
)