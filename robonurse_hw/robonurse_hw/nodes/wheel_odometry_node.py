#!/usr/bin/env python3
"""wheel_odometry_node.py

Reads binary telemetry packets from Arduino Mega and publishes nav_msgs/Odometry.
Telemetry packet format (Arduino -> host):
[0x55, 0xAA] header (2 bytes) | pkt_id (1 byte) | payload_len (1 byte) | payload | crc8 (1 byte)

pkt_id = 0x10 : odometry packet
  payload: int32 seq | int32 t_ms | int32 enc0 | int32 enc1 | int32 enc2 | int32 enc3 | int16 battery_mv

The node parses packets, computes velocities and publishes /odom (frame_id: odom, child_frame_id: base_link).
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import serial
import struct
import time
import math
from collections import deque

def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry_node')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('wheel_radius_m', 0.05)
        self.declare_parameter('encoder_ticks_per_rev', 1024)
        self.declare_parameter('wheel_base_m', 0.3)
        self.declare_parameter('track_width_m', 0.3)

        self.port = str(self.get_parameter('serial_port').value)
        self.baud = int(self.get_parameter('baudrate').value)
        self.r = float(self.get_parameter('wheel_radius_m').value)
        self.ticks = int(self.get_parameter('encoder_ticks_per_rev').value)
        self.wheel_base = float(self.get_parameter('wheel_base_m').value)
        self.track = float(self.get_parameter('track_width_m').value)

        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0.1)
            self.get_logger().info(f"Opened serial port {self.port} @ {self.baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            raise

        self.buffer = bytearray()
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.last_enc = None
        self.last_time = None

        # read loop timer
        self.create_timer(0.02, self.read_loop)

    def read_loop(self):
        try:
            data = self.ser.read(256)
            if data:
                self.buffer.extend(data)
                self.parse_buffer()
        except Exception as e:
            self.get_logger().error(f"Serial read error: {e}")

    def parse_buffer(self):
        # look for header 0x55 0xAA
        while True:
            if len(self.buffer) < 4:
                return
            # find header
            hdr_idx = None
            for i in range(len(self.buffer)-1):
                if self.buffer[i] == 0x55 and self.buffer[i+1] == 0xAA:
                    hdr_idx = i
                    break
            if hdr_idx is None:
                # discard
                self.buffer.clear()
                return
            if hdr_idx > 0:
                del self.buffer[:hdr_idx]
            if len(self.buffer) < 4:
                return
            pkt_id = self.buffer[2]
            payload_len = self.buffer[3]
            total_len = 2 + 1 + 1 + payload_len + 1
            if len(self.buffer) < total_len:
                return
            pkt = bytes(self.buffer[:total_len])
            # verify crc
            crc_calc = crc8(pkt[:-1])
            crc_recv = pkt[-1]
            if crc_calc != crc_recv:
                # drop header and continue
                del self.buffer[0:2]
                continue
            # parse payload
            payload = pkt[4:-1]
            self.handle_packet(pkt_id, payload)
            del self.buffer[:total_len]

    def handle_packet(self, pkt_id, payload: bytes):
        if pkt_id == 0x10 and len(payload) >= 4+4+4*4+2:
            # int32 seq, int32 t_ms, 4x int32 enc, int16 batt_mv
            seq, t_ms, enc0, enc1, enc2, enc3, batt_mv = struct.unpack('<iiiiiih', payload[:4+4+4*4+2])
            now = time.time()
            # compute dt
            if self.last_enc is None:
                self.last_enc = (enc0, enc1, enc2, enc3)
                self.last_time = now
                return
            dt = now - self.last_time
            if dt <= 0:
                return
            d_enc = [enc0 - self.last_enc[0], enc1 - self.last_enc[1], enc2 - self.last_enc[2], enc3 - self.last_enc[3]]
            # convert ticks -> meters: ticks / ticks_per_rev * 2*pi*r
            meters = [ (d / self.ticks) * 2 * math.pi * self.r for d in d_enc ]
            # simple kinematic inverse for mecanum -> vx, vy, wz estimation (approx)
            # using standard mecanum forward kinematics
            vx = (meters[0] + meters[1] + meters[2] + meters[3]) / 4.0 / dt
            vy = (-meters[0] + meters[1] + meters[2] - meters[3]) / 4.0 / dt
            wz = (-meters[0] + meters[1] - meters[2] + meters[3]) / (4.0 * (self.wheel_base + self.track) / 2.0) / dt

            odom = Odometry()
            odom.header.stamp = self.get_clock().now().to_msg()
            odom.header.frame_id = 'odom'
            odom.child_frame_id = 'base_link'
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = wz
            # pose unknown here (no pose integration); set zeros
            odom.pose.pose.position.x = 0.0
            odom.pose.pose.position.y = 0.0
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)

            self.last_enc = (enc0, enc1, enc2, enc3)
            self.last_time = now

def main(args=None):
    rclpy.init(args=args)
    node = WheelOdometryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down wheel_odometry_node')
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
