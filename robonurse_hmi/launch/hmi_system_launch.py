from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Launches all necessary nodes for the RoboNurse HMI system."""
    
    # 1. Teleop Bridge Node (HMI Gateway)
    teleop_bridge_node = Node(
        package='robonurse_hmi',
        executable='teleop_bridge_node',
        name='teleop_bridge_node',
        output='screen'
    )

    # 2. Task Manager Node (System Brain/FSM)
    task_manager_node = Node(
        package='robonurse_hmi',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen'
    )
    
    # 3. Actuator Control Node (Hardware Interface)
    actuator_control_node = Node(
        package='robonurse_hmi',
        executable='actuator_control_node',
        name='actuator_control_node',
        output='screen'
    )
    
    # 4. ROS Bridge Node (Crucial for Web HMI connectivity)
    # NOTE: You must have the rosbridge_server package installed for this to work.
    ros_bridge_node = Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[
            {'port': 9090} # Standard port
        ]
    )

    return LaunchDescription([
        teleop_bridge_node,
        task_manager_node,
        actuator_control_node,
        ros_bridge_node
    ])