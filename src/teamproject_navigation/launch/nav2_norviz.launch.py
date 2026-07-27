# nav2_norviz.launch.py — RViz "없는" Nav2 기동 (2026-07-04)
# =====================================================================
# 왜: turtlebot3_navigation2/navigation2.launch.py 는 rviz 를 '무조건' 띄움
#     (rviz:=false 인자는 선언돼 있지 않아 조용히 무시됨!).
#     그래서 T3(rviz_keepout, RViz 담당)와 같이 쓰면 RViz 가 2개 뜨던 것.
# 해결: nav2_bringup 을 직접 include (TB3 런처와 동일한 실체) + rviz 미포함.
#
# 사용(T2 대체):
#   ros2 launch teamproject_navigation nav2_norviz.launch.py
#   (map/params 기본값이 우리 실사용 파일. 바꾸려면 map:=... params_file:=...)
# RViz 는 T3(rviz_keepout.launch.py) 하나만 띄운다.
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    home = os.path.expanduser('~')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(home, 'team_ws', 'maps', 'map.yaml'),
        description='맵 yaml')
    params_arg = DeclareLaunchArgument(
        'params_file', default_value=os.path.join(home, 'nav_params', 'burger_rpp.yaml'),
        description='Nav2 파라미터')
    sim_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': LaunchConfiguration('params_file'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    return LaunchDescription([map_arg, params_arg, sim_arg, nav2])
