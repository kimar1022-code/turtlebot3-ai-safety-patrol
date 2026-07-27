#!/usr/bin/env python3
"""
robot_bringup_nocam.launch.py — 카메라 빼고 로봇 기동 (모터 + LiDAR + IMU만).
커스텀 robot_bringup.launch.py 에서 camera.launch.py / republish 만 제거한 버전.
목적: 라즈베리파이 발열·CPU 부하 ↓ (주행 테스트용).

사용 (로봇/Pi 에서):
  ros2 launch teamproject_robot_bringup robot_bringup_nocam.launch.py

배치:
  ~/turtlebot3_ws/src/teamproject_robot_bringup/launch/ 에 복사 →
  setup.py(또는 CMakeLists)의 launch 설치목록에 이 파일 추가 →
  cd ~/turtlebot3_ws && colcon build --packages-select teamproject_robot_bringup && source install/setup.bash

전제:
  - LiDAR 모델은 환경변수로 지정됨: export LDS_MODEL=LDS-03 (로봇 ~/.bashrc)
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    tb3_bringup = FindPackageShare('turtlebot3_bringup')

    return LaunchDescription([
        # 모터 · LiDAR(LDS-03, 환경변수로 자동) · IMU  — 카메라 없음
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([tb3_bringup, 'launch', 'robot.launch.py'])
            ),
        ),
    ])
