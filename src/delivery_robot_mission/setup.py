from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_robot_mission'

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
            'launch/mission.launch.py',
            'launch/full_system_launch.py'
        ]),
        # 설정 파일
        (os.path.join('share', package_name, 'config'), 
         glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
    ],
    zip_safe=True,
    maintainer='배달로봇팀',
    maintainer_email='robotics.team@university.ac.kr',
    description='배달 로봇용 임무 관리 시스템 - 외부 API 연동, 경로 생성',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 임무 제어 노드
            'mission_control_node = delivery_robot_mission.mission_control_node:main',
            # 시스템 모니터 노드
            'system_monitor_node = delivery_robot_mission.system_monitor_node:main',
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
        'Topic :: Scientific/Engineering :: GIS',
        'Topic :: Software Development :: Libraries :: Python Modules',
    ],
    python_requires='>=3.8',
)