from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_api_control',
            executable='main_client',
            name='main_client_node',
            output='screen'
        ),
        Node(
            package='voice_processing',
            executable='get_keyword',
            name='get_keyword_node',
            output='screen'
        ),
        Node(
            package='OBB',
            executable='detection',
            name='object_detection_node',
            output='screen'
        ),
    ])
