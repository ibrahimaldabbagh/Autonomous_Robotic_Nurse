from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    cfg = os.path.join(os.path.dirname(__file__), '..', 'config', 'status_map.yaml')
    return LaunchDescription([
        Node(
            package='robonurse_patient_status',
            executable='patient_status_node',
            name='patient_status_node',
            output='screen',
            parameters=[cfg]
        )
    ])
