from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'camera_params.yaml')
    return LaunchDescription([
        Node(
            package='robonurse_perception',
            executable='perception_camera_node',
            name='perception_camera_node',
            output='screen',
            parameters=[cfg]
        )
    ])
