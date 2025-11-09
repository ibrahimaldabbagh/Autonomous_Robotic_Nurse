import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class ActuatorControlNode(Node):
    """
    Simulates the low-level motor controller and hardware interface.
    It takes commands from high-level nodes and "executes" them.
    """
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # --- Subscriptions (Actuator Commands) ---
        # Receives base velocity from Teleop Bridge or Nav2
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        # Receives commands relayed from the Teleop Bridge
        self.create_subscription(String, '/system/manipulator_action', self.manipulator_callback, 1)
        self.create_subscription(String, '/system/head_action', self.head_callback, 1)
        
        self.get_logger().info('Actuator Control Node initialized, ready to receive commands.')

    def cmd_vel_callback(self, msg: Twist):
        """Handles motor commands for the base."""
        lin_x = msg.linear.x
        ang_z = msg.angular.z
        
        if abs(lin_x) > 0.0 or abs(ang_z) > 0.0:
            self.get_logger().info(f"MOTOR EXECUTION: Linear={lin_x:.2f} m/s, Angular={ang_z:.2f} rad/s")
        # else: It's silent when stopped (Twist is 0,0)

    def manipulator_callback(self, msg: String):
        """Handles manipulator actions (load/unload food/meds)."""
        action = msg.data.lower().strip()
        
        if action == 'load_food':
            self.get_logger().warn("MANIPULATOR: Executing Food Loading Sequence...")
        elif action == 'load_meds':
            self.get_logger().warn("MANIPULATOR: Executing Medicine Loading Sequence...")
        else:
            self.get_logger().error(f"MANIPULATOR: Unknown action received: {action}")

    def head_callback(self, msg: String):
        """Handles head/camera movement."""
        position = msg.data.lower().strip()
        self.get_logger().info(f"HEAD MOVEMENT: Adjusting camera to position: {position.upper()}")

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()