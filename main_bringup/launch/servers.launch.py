# 파일명: launch/all.launch.py

import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # OBB 패키지: object detection 노드
        Node(
            package='OBB',
            executable='detection',
            name='obb_detection',
            output='screen'
        ),

        # OBB 패키지: cancel object 서비스 노드
        Node(
            package='OBB',
            executable='canel',       # typo 없이 실제 executable 이름 확인
            name='obb_cancel',
            output='screen'
        ),

        # OBB 패키지: is_adult 서비스 노드
        Node(
            package='OBB',
            executable='is_adult',
            name='obb_is_adult',
            output='screen'
        ),

        # gui 패키지: real GUI 노드
        Node(
            package='gui',
            executable='real_gui',
            name='gui_real',
            output='screen'
        ),

        # robot_api_control 패키지: control 노드
        Node(
            package='robot_api_control',
            executable='control',
            name='robot_control',
            output='screen'
        ),
    ])
