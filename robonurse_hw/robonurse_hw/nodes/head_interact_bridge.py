import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16, String
from rclpy.qos import qos_profile_system_default

# NOTE: This node publishes topics expected by the Arduino Nano through ROSSERIAL.

class HeadInteractBridge(Node):
    def __init__(self):
        super().__init__('head_interact_bridge')

        # --- Publishers (Commands TO Arduino Nano) ---
        # The topics MUST match the subscriber names in nano_head_control.ino
        self.pan_pub = self.create_publisher(Int16, 'head/pan_command', 10)
        self.tilt_pub = self.create_publisher(Int16, 'head/tilt_command', 10)
        self.sound_pub = self.create_publisher(Int16, 'head/sound_command', 10)

        # --- Subscriptions (Feedback FROM Arduino Nano) ---
        self.pan_feedback_sub = self.create_subscription(
            Int16,
            'head/pan_status',
            self.pan_status_callback,
            10
        )
        self.get_logger().info('Head Interact Bridge initialized.')

    def pan_status_callback(self, msg: Int16):
        """Handles feedback from the Nano about the current pan position."""
        self.get_logger().debug(f'Head Pan Status: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    node = HeadInteractBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()