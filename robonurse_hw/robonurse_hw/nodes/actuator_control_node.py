import rclpy
from rclpy.node import Node
from robonurse_interfaces.srv import DispenseItem
import serial
import struct

# Serial Protocol Commands
ACTUATOR_CMD = 0x03
ACTUATOR_ACK = 0x04

class ActuatorControlNode(Node):
    def __init__(self):
        super().__init__('actuator_control_node')
        
        # --- Parameters ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        
        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value

        # --- Serial Setup ---
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=3.0) # Longer timeout for actuator operation
            self.get_logger().info(f'Actuator: Successfully opened serial port: {serial_port}')
        except serial.SerialException as e:
            self.get_logger().error(f"Actuator: Failed to open serial port {serial_port}: {e}")
            self.ser = None

        # --- Service Server ---
        self.dispense_service = self.create_service(
            DispenseItem,
            '/dispense/item',
            self.dispense_callback
        )
        self.get_logger().info('Actuator service /dispense/item ready.')

    def dispense_callback(self, request, response):
        """Callback for the DispenseItem service."""
        
        if self.ser is None:
            response.success = False
            response.message = "Serial port not open."
            return response
        
        self.get_logger().info(f'Received dispense request: Item {request.item_id}, Slot {request.slot_id}')

        # 1. Send Command (0x03) to Arduino Mega
        header_cmd = struct.pack('<BB', 0xFF, ACTUATOR_CMD)
        payload = struct.pack('<Bi', request.item_id, request.slot_id) # uint8 + int32
        packet = header_cmd + payload
        
        try:
            self.ser.write(packet)
        except serial.SerialException as e:
            response.success = False
            response.message = f"Error sending serial command: {e}"
            return response
            
        # 2. Wait for Acknowledgement (0x04) from Arduino Mega
        # Since the Mega firmware handles the actual physical movement, we must wait for its ACK
        
        start_time = self.get_clock().now().nanoseconds / 1e9
        success = False
        ack_item_id = 0
        
        # Poll for ACK packet for up to the serial timeout (3.0s)
        while (self.get_clock().now().nanoseconds / 1e9 - start_time) < self.ser.timeout:
            if self.ser.in_waiting >= 4: # Minimum ACK packet size (Header, CMD, Success, ItemID)
                if self.ser.read() == b'\xFF':
                    if self.ser.in_waiting >= 3:
                        cmd_id = struct.unpack('<B', self.ser.read())[0]
                        if cmd_id == ACTUATOR_ACK:
                            ack_payload = self.ser.read(2) # bool + uint8
                            success_byte, ack_item_id_byte = struct.unpack('<BB', ack_payload)
                            
                            success = bool(success_byte)
                            ack_item_id = ack_item_id_byte
                            break # Found and processed ACK
            
            # Simple sleep to avoid busy-waiting the CPU (not necessary with serial.timeout but good practice)
            time.sleep(0.01)

        # 3. Form Response
        if success and ack_item_id == request.item_id:
            response.success = True
            response.message = f"Dispense of Item {ack_item_id} completed successfully."
        else:
            response.success = False
            response.message = "Dispense failed or timed out waiting for Arduino acknowledgement."

        return response

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()