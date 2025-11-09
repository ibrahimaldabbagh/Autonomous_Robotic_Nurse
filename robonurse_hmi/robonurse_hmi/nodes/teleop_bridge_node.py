import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TeleopBridgeNode(Node):
    """
    Acts as the bridge and gatekeeper for all commands coming from the Web HMIs.
    It checks the robot's current mode before relaying commands internally.
    """
    def __init__(self):
        super().__init__('teleop_bridge_node')
        
        # --- State (Mode is typically managed by a Task Manager/FSM) ---
        self.current_mode = 'autonomous' 
        self.last_teleop_cmd = Twist()

        # --- Subscriptions from Operator HMI (via ROS Bridge) ---
        self.create_subscription(Twist, '/joy_vel', self.teleop_command_callback, 10)
        self.create_subscription(String, '/robot/mode_switch', self.mode_switch_callback, 1)
        self.create_subscription(String, '/manipulator/control', self.manipulator_command_callback, 1)
        self.create_subscription(String, '/head/control', self.head_command_callback, 1)

        # --- Subscriptions from Patient HMI (via ROS Bridge) ---
        self.create_subscription(String, '/patient/emergency', self.patient_emergency_callback, 1)
        self.create_subscription(String, '/patient/request_nurse', self.patient_nurse_request_callback, 1)
        
        # --- Internal Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.manipulator_pub = self.create_publisher(String, '/system/manipulator_action', 1)
        self.head_pub = self.create_publisher(String, '/system/head_action', 1)
        self.emergency_pub = self.create_publisher(String, '/system/emergency_trigger', 1)
        self.mode_control_pub = self.create_publisher(String, '/system/mode_control', 1) # Internal mode change request
        
        # Timer to continuously publish velocity in manual mode
        self.timer = self.create_timer(0.1, self.publish_velocity)
        
        self.get_logger().info('Teleop Bridge Node initialized.')

    # --- Mode Management ---

    def mode_switch_callback(self, msg: String):
        """Relays the mode change request internally to the Task Manager."""
        new_mode = msg.data.lower().strip()
        if new_mode in ['manual', 'autonomous']:
            self.current_mode = new_mode # Update local state immediately for command gating
            self.mode_control_pub.publish(msg)
            self.get_logger().warn(f'Mode switch request sent: {new_mode.upper()}')
        else:
            self.get_logger().error(f"Invalid mode command: {new_mode}")

    # --- Operator Teleop Commands (Gated by Manual Mode) ---

    def teleop_command_callback(self, msg: Twist):
        """Stores velocity command; publishes only if in manual mode."""
        self.last_teleop_cmd = msg

    def publish_velocity(self):
        """Continuously publishes stored velocity if in manual mode."""
        if self.current_mode == 'manual':
            self.cmd_vel_pub.publish(self.last_teleop_cmd)
        else:
            # Send zero velocity when not in manual mode (to ensure motor safety)
            zero_twist = Twist()
            zero_twist.linear.x = 0.0
            zero_twist.angular.z = 0.0
            self.cmd_vel_pub.publish(zero_twist)

    def manipulator_command_callback(self, msg: String):
        """Relays manipulator command only if in manual mode."""
        if self.current_mode == 'manual':
            self.manipulator_pub.publish(msg)
            self.get_logger().info(f"Manual Manipulator Action: {msg.data}")
        else:
            self.get_logger().warn(f"Manipulator command blocked: {msg.data}")

    def head_command_callback(self, msg: String):
        """Relays head command only if in manual mode."""
        if self.current_mode == 'manual':
            self.head_pub.publish(msg)
            self.get_logger().info(f"Manual Head Action: {msg.data}")
        else:
            self.get_logger().warn(f"Head command blocked: {msg.data}")

    # --- Patient Service Requests (Always allowed) ---

    def patient_emergency_callback(self, msg: String):
        """Immediately publishes an emergency trigger for critical response."""
        self.emergency_pub.publish(msg)
        self.get_logger().fatal(f"!!! EMERGENCY TRIGGERED by Patient !!! Data: {msg.data}")
        # Note: In a real system, this would also trigger a full system shutdown/safe stop.
    
    def patient_nurse_request_callback(self, msg: String):
        """Publishes request to the Task Manager for automated nurse summoning."""
        # The Task Manager would handle calling the human nurse or moving the robot.
        self.mode_control_pub.publish(String(data="service_nurse_request"))
        self.get_logger().info(f"Patient requested Nurse. Data: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = TeleopBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()