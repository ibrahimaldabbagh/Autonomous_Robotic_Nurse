#!/usr/bin/env python3
"""motor_control_node.py

Subscribes to /cmd_vel (geometry_msgs/Twist), converts to wheel velocities for an omnidirectional mecanum robot
(but adapted here for 4 independent wheels; user will tune kinematics if differential or mecanum).
Sends binary command packets over serial to Arduino Mega.

Packet format (host -> Arduino):
[0xAA, 0x55] (2 bytes header) | cmd_id (1 byte) | payload_len (1 byte) | payload (N bytes) | crc8 (1 byte)

cmd_id = 0x01 : set wheel speeds (payload: 4 x int16 RPM or mm/s)
cmd_id = 0x02 : stop motors (no payload)
cmd_id = 0x03 : keepalive / ping (no payload)

The node implements a watchdog: if no /cmd_vel received within watchdog_timeout, sends stop.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct
import time
import math
import os
from rclpy.parameter import Parameter

def crc8(data: bytes) -> int:
    # simple CRC-8 (CRC-8-ATM polynomial 0x07)
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        # parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('wheel_base_m', 0.3)
        self.declare_parameter('track_width_m', 0.3)
        self.declare_parameter('encoder_ticks_per_rev', 1024)
        self.declare_parameter('max_linear_speed', 0.8)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('watchdog_timeout', 0.5)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.r = float(self.get_parameter('wheel_radius_m').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.track = float(self.get_parameter('track_width_m').value)
        self.encoder_ticks = int(self.get_parameter('encoder_ticks_per_rev').value)
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        self.watchdog_timeout = float(self.get_parameter('watchdog_timeout').value)

        # serial
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened serial port {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        # subscriber
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 10)

        self.last_cmd_time = time.time()

        # timer to send keepalive / watchdog enforcement
        self.create_timer(0.05, self.timer_cb)

    def cmd_cb(self, msg: Twist):
        # clamp velocities
        vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        wz = max(-self.max_ang, min(self.max_ang, msg.angular.z))

        # Convert to wheel speeds for 4-wheel omni (mecanum) - wheel order: FL, FR, RL, RR
        # Standard mecanum inverse kinematics for vx, vy, wz:
        # v_fl = (1/r) * (vx - vy - (wheel_base + track_width)/2 * wz)
        L = self.wheel_base
        W = self.track
        r = self.r
        # linear velocities at wheel (m/s)
        v_fl = vx - vy - (L + W)/2.0 * wz
        v_fr = vx + vy + (L + W)/2.0 * wz
        v_rl = vx + vy - (L + W)/2.0 * wz
        v_rr = vx - vy + (L + W)/2.0 * wz

        # convert to mm/s integers for payload
        wheels = [v_fl, v_fr, v_rl, v_rr]
        wheels_mm_s = [int(max(-32000, min(32000, int(v*1000)))) for v in wheels]  # mm/s clamped to int16 range

        # build payload: 4 x int16 little-endian
        payload = struct.pack('<hhhh', *wheels_mm_s)

        # assemble packet
        header = b'\xAA\x55'
        cmd_id = b'\x01'
        payload_len = struct.pack('B', len(payload))
        packet_no_crc = header + cmd_id + payload_len + payload
        crc = struct.pack('B', crc8(packet_no_crc))
        packet = packet_no_crc + crc

        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

        self.last_cmd_time = time.time()

    def send_stop(self):
        header = b'\xAA\x55'
        cmd_id = b'\x02'
        payload_len = b'\x00'
        packet_no_crc = header + cmd_id + payload_len
        crc = struct.pack('B', crc8(packet_no_crc))
        packet = packet_no_crc + crc
        try:
            self.ser.write(packet)
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def timer_cb(self):
        now = time.time()
        if (now - self.last_cmd_time) > self.watchdog_timeout:
            # send stop if not already
            self.send_stop()
        else:
            # send keepalive ping to Arduino every 200ms
            header = b'\xAA\x55'
            cmd_id = b'\x03'
            payload_len = b'\x00'
            packet_no_crc = header + cmd_id + payload_len
            crc = struct.pack('B', crc8(packet_no_crc))
            packet = packet_no_crc + crc
            try:
                self.ser.write(packet)
            except Exception:
                pass

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down motor_control_node')
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
