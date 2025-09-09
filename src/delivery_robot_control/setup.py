from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_robot_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
        # 설정 파일
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy>=1.21.0',         # NumPy
        'pyserial>=3.5',         # 시리얼 통신 (Arduino/마이크로컨트롤러용)
        'RPi.GPIO; platform_machine=="armv7l"',  # Raspberry Pi GPIO (ARM용만)
    ],
    zip_safe=True,
    maintainer='배달로봇팀',
    maintainer_email='robotics.team@university.ac.kr',
    description='배달 로봇용 제어 시스템 - ros2_control 하드웨어 인터페이스',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 하드웨어 인터페이스 노드
            'hardware_interface_node = delivery_robot_control.hardware_interface_node:main',
            # 모터 컨트롤러 노드
            'motor_controller_node = delivery_robot_control.motor_controller_node:main',
            # GPIO 제어 노드
            'gpio_controller_node = delivery_robot_control.gpio_controller_node:main',
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
        'Topic :: System :: Hardware :: Hardware Drivers',
        'Topic :: Scientific/Engineering :: Electronic Design Automation (EDA)',
    ],
    python_requires='>=3.8',
)