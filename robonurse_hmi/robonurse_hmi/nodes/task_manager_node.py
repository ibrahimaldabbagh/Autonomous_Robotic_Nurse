import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TaskManagerNode(Node):
    """
    Manages the robot's high-level state (Autonomous vs. Manual) and executes 
    patient service requests or autonomous patrol/navigation tasks.
    """
    def __init__(self):
        super().__init__('task_manager_node')
        
        # --- State ---
        self.robot_state = 'INITIALIZING'
        
        # --- Subscriptions (Internal Control) ---
        # Receives mode change requests from the Teleop Bridge
        self.create_subscription(String, '/system/mode_control', self.mode_control_callback, 1)
        # Receives emergency signals (highest priority)
        self.create_subscription(String, '/system/emergency_trigger', self.emergency_callback, 1)

        # --- Publishers (Internal Control) ---
        # The node that controls navigation (e.g., Nav2 interface)
        self.navigation_pub = self.create_publisher(String, '/navigation/goal', 1) 
        # Publishes overall robot status back to the HMI (not implemented here, but necessary)
        # self.status_pub = self.create_publisher(String, '/hmi/system_status', 1)
        
        # Timer for autonomous behavior simulation
        self.autonomous_timer = self.create_timer(5.0, self.autonomous_loop)
        
        self.set_state('AUTONOMOUS')

    def set_state(self, new_state):
        """Helper to manage state transitions."""
        self.robot_state = new_state
        self.get_logger().info(f"Robot State changed to: {self.robot_state}")
        # In a real system, this would publish the state to /hmi/system_status

    def mode_control_callback(self, msg: String):
        """Handles mode changes and service requests."""
        data = msg.data.lower().strip()
        
        if data in ['manual', 'autonomous']:
            self.set_state(data.upper())
            if data == 'manual':
                self.get_logger().warn("Autonomous tasks paused. Operator has MANUAL control.")
            elif data == 'autonomous':
                self.get_logger().info("Returning to autonomous patrol and service monitoring.")
        
        elif data == 'service_nurse_request':
            if self.robot_state == 'MANUAL':
                 self.get_logger().error("Cannot run service: Robot is in MANUAL mode.")
            else:
                self.set_state('SERVICE_NURSE')
                # Start navigation sequence to the Nurse Station or nearest staff member
                self.navigation_pub.publish(String(data="GoToNurseStation"))
                self.get_logger().info("Initiating Nurse Request Service (Navigation).")
        
        # Add logic for other patient service requests here (e.g., 'service_cafe_order')

    def emergency_callback(self, msg: String):
        """Immediately halts all non-emergency functions."""
        self.set_state('EMERGENCY_STOP')
        self.get_logger().fatal(f"CRITICAL: System halted by emergency signal: {msg.data}")
        # Send full zero velocity command immediately to /cmd_vel if not already handled by a lower node
        
    def autonomous_loop(self):
        """Simulates autonomous behavior when in the AUTONOMOUS state."""
        if self.robot_state == 'AUTONOMOUS':
            # This is where a more complex behavior tree or FSM would live
            self.get_logger().debug("Executing Autonomous Patrol Loop...")
            # For simulation: periodically check internal queues, or send simple patrol goals

def main(args=None):
    rclpy.init(args=args)
    node = TaskManagerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()