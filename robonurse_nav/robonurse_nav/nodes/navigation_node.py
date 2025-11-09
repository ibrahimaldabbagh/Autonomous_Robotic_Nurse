from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

def generate_launch_description():
    # Package directories
    nav2_bringup_dir = FindPackageShare('nav2_bringup').find('nav2_bringup')
    robonurse_nav_dir = FindPackageShare('robonurse_nav').find('robonurse_nav')
    
    # Configuration file path
    nav2_params_file = PathJoinSubstitution([robonurse_nav_dir, 'params', 'nav2_params.yaml'])
    
    # Placeholder Map file (you must create this .yaml and .pgm file)
    map_file_path = LaunchConfiguration('map', default=os.path.join(
        robonurse_nav_dir, 'maps', 'hospital_corridor.yaml'))

    return LaunchDescription([
        # --- 1. Nav2 Stack (Main Components: AMCL, Map Server, Lifecycle Manager) ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([nav2_bringup_dir, 'launch', 'bringup_launch.py'])
            ),
            launch_arguments={
                'map': map_file_path,
                'params_file': nav2_params_file,
                'use_lifecycle_mgr': 'true'
            }.items(),
        ),
        
        # --- 2. Custom Local Planner Node ---
        # Runs your safety-checked controller that subscribes to Nav2's output and publishes /cmd_vel
        Node(
            package='robonurse_nav',
            executable='local_planner_node.py',
            name='local_planner_node',
            output='screen',
            parameters=[nav2_params_file]
        ),
        
        # --- 3. (Optional) Rviz for Visualization (useful for debugging on Pi) ---
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen'
        )
    ])