import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
import math
import numpy as np

# A simplified local controller that would run alongside the Nav2 controller 
# or completely replace it for competition simplicity.
class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner_node')
        
        # --- Parameters ---
        self.declare_parameter('ir_stop_distance', 0.3) # m
        self.stop_dist = self.get_parameter('ir_stop_distance').get_parameter_value().double_value

        # --- Subscriptions ---
        # The low-level controller needs range sensor data for immediate halt
        self.ir_sub = self.create_subscription(
            Range,
            '/sensor/ir',
            self.ir_callback,
            10
        )
        # The low-level controller needs velocity commands from the high-level planner
        self.nav_cmd_sub = self.create_subscription(
            Twist,
            '/nav_cmd_vel', # Nav2 DWB planner output topic
            self.nav_cmd_callback,
            10
        )
        
        # --- Publishers ---
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10) # To the motor controller
        
        # --- State ---
        self.obstacle_close = False
        self.last_nav_cmd = Twist()

    def ir_callback(self, msg: Range):
        """Update obstacle status based on IR sensor."""
        # Simple stop check: if any range sensor is too close, flag an obstacle.
        if msg.range < self.stop_dist:
            self.obstacle_close = True
        elif msg.range > self.stop_dist * 1.5: # Simple hysteresis to re-start
            self.obstacle_close = False
    
    def nav_cmd_callback(self, msg: Twist):
        """Receives command from the high-level Nav2 controller."""
        self.last_nav_cmd = msg
        self.publish_safe_cmd()

    def publish_safe_cmd(self):
        """Checks for obstacles and publishes a safe command."""
        
        cmd_vel = self.last_nav_cmd
        
        if self.obstacle_close:
            # Override: Stop the robot immediately
            cmd_vel = Twist()
            cmd_vel.linear.x = 0.0
            cmd_vel.linear.y = 0.0
            cmd_vel.angular.z = 0.0
            self.get_logger().warn("IR/Ultrasonic Obstacle detected. Publishing zero velocity.")
        
        # Publish the command (either the planned one or the zero-override)
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()