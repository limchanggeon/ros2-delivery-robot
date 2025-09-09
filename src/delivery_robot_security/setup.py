from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'delivery_robot_security'

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
    ],
    zip_safe=True,
    maintainer='배달로봇팀',
    maintainer_email='robotics.team@university.ac.kr',
    description='배달 로봇용 보안 시스템 - QR 코드 인증, 도어 제어',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # QR 코드 인증 노드
            'authentication_node = delivery_robot_security.authentication_node:main',
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
        'Topic :: Scientific/Engineering :: Image Recognition',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: System :: Hardware :: Hardware Drivers',
    ],
    python_requires='>=3.8',
)