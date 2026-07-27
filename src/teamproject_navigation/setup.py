from setuptools import find_packages, setup

package_name = 'teamproject_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/waypoints.yaml']),
        ('share/' + package_name + '/launch', [
            'launch/robot_nodes.launch.py',
            'launch/robot_bringup.launch.py',
            'launch/keepout_filter.launch.py',
            'launch/keepout_filter_newmap.launch.py',
            'launch/rviz_keepout.launch.py',
            'launch/nav2_norviz.launch.py',   # ★rviz 없는 Nav2 (TB3 런처가 rviz 무조건 띄우는 문제 대체)
        ]),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    zip_safe=True,
    maintainer='ar',
    maintainer_email='kimar1022@gmail.com',
    description='Factory patrol robot navigation (FSM, status report, dispatch)',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'patrol_commander=teamproject_navigation.patrol_commander:main',
            'status_reporter=teamproject_navigation.status_reporter:main',
            'manual_cmd_gate=teamproject_navigation.manual_cmd_gate:main',
        ],
    },
)