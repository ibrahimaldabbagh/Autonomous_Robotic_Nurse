#!/usr/bin/env python3
"""video_stream_node.py

ROS2 node for video streaming to operator interface.
Supports:
  - H.264 encoding via OpenCV/FFmpeg
  - RTSP streaming (recommended for low latency)
  - WebRTC streaming (for browser-based operator interface)
  - Recording clips on-demand via service calls

Subscribes:
  - /camera/image_raw (sensor_msgs/Image)

Services:
  - /video/start_recording (std_srvs/Trigger) - Start recording clip
  - /video/stop_recording (std_srvs/Trigger) - Stop recording clip

Parameters:
  - stream_url: RTSP URL to stream to (e.g., rtsp://0.0.0.0:8554/robonurse)
  - enable_webrtc: Enable WebRTC signaling server
  - webrtc_port: WebRTC signaling port (default: 8080)
  - codec: h264 or h265
  - bitrate_kbps: Target bitrate
  - fps: Stream FPS
  - width: Stream width
  - height: Stream height
  - recordings_dir: Directory to save recordings
  - max_clip_duration_sec: Maximum clip duration

Author: Ibrahim Aldabbagh
Email: eng.ibrahim.aldabbagh@gmail.com
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2
import numpy as np
import subprocess
import threading
import time
import os
from datetime import datetime
import queue


class VideoStreamNode(Node):
    def __init__(self):
        super().__init__('video_stream_node')
        
        # Parameters
        self.declare_parameter('stream_url', 'rtsp://0.0.0.0:8554/robonurse')
        self.declare_parameter('enable_webrtc', False)
        self.declare_parameter('webrtc_port', 8080)
        self.declare_parameter('codec', 'h264')
        self.declare_parameter('bitrate_kbps', 2000)
        self.declare_parameter('fps', 15)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('recordings_dir', '/tmp/robonurse_recordings')
        self.declare_parameter('max_clip_duration_sec', 60.0)
        self.declare_parameter('buffer_size', 30)
        
        self.stream_url = str(self.get_parameter('stream_url').value)
        self.enable_webrtc = bool(self.get_parameter('enable_webrtc').value)
        self.webrtc_port = int(self.get_parameter('webrtc_port').value)
        self.codec = str(self.get_parameter('codec').value)
        self.bitrate = int(self.get_parameter('bitrate_kbps').value)
        self.fps = int(self.get_parameter('fps').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.recordings_dir = str(self.get_parameter('recordings_dir').value)
        self.max_clip_duration = float(self.get_parameter('max_clip_duration_sec').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        
        # Create recordings directory
        os.makedirs(self.recordings_dir, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Frame queue for streaming thread
        self.frame_queue = queue.Queue(maxsize=self.buffer_size)
        
        # Recording state
        self.recording = False
        self.video_writer = None
        self.recording_start_time = None
        self.current_recording_path = None
        
        # FFmpeg process for RTSP streaming
        self.ffmpeg_process = None
        self.streaming = False
        
        # Start streaming thread
        self.streaming_thread = threading.Thread(target=self._streaming_worker, daemon=True)
        self.streaming_thread.start()
        
        # Services
        self.srv_start_rec = self.create_service(
            Trigger, 
            '/video/start_recording', 
            self.start_recording_callback
        )
        self.srv_stop_rec = self.create_service(
            Trigger, 
            '/video/stop_recording', 
            self.stop_recording_callback
        )
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # WebRTC signaling server (simplified)
        if self.enable_webrtc:
            self._start_webrtc_server()
        
        self.get_logger().info(f'Video stream node started')
        self.get_logger().info(f'Streaming to: {self.stream_url}')
        self.get_logger().info(f'Recordings saved to: {self.recordings_dir}')
        
    def _start_ffmpeg_stream(self):
        """Start FFmpeg process for RTSP streaming"""
        try:
            # FFmpeg command for RTSP streaming with H.264
            cmd = [
                'ffmpeg',
                '-y',  # Overwrite output
                '-f', 'rawvideo',
                '-pixel_format', 'bgr24',
                '-video_size', f'{self.width}x{self.height}',
                '-framerate', str(self.fps),
                '-i', 'pipe:0',  # Input from stdin
                '-c:v', 'libx264',
                '-preset', 'ultrafast',
                '-tune', 'zerolatency',
                '-b:v', f'{self.bitrate}k',
                '-maxrate', f'{self.bitrate}k',
                '-bufsize', f'{self.bitrate*2}k',
                '-pix_fmt', 'yuv420p',
                '-g', str(self.fps * 2),  # Keyframe interval
                '-f', 'rtsp',
                self.stream_url
            ]
            
            self.ffmpeg_process = subprocess.Popen(
                cmd,
                stdin=subprocess.PIPE,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.streaming = True
            self.get_logger().info('FFmpeg RTSP stream started')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start FFmpeg: {e}')
            self.streaming = False
    
    def _streaming_worker(self):
        """Background thread to handle streaming"""
        self._start_ffmpeg_stream()
        
        while rclpy.ok():
            try:
                # Get frame from queue with timeout
                frame = self.frame_queue.get(timeout=1.0)
                
                if self.streaming and self.ffmpeg_process and self.ffmpeg_process.poll() is None:
                    try:
                        # Write frame to FFmpeg stdin
                        self.ffmpeg_process.stdin.write(frame.tobytes())
                    except BrokenPipeError:
                        self.get_logger().error('FFmpeg pipe broken, restarting...')
                        self._start_ffmpeg_stream()
                else:
                    # Restart FFmpeg if it died
                    if self.ffmpeg_process and self.ffmpeg_process.poll() is not None:
                        self.get_logger().warn('FFmpeg process died, restarting...')
                        self._start_ffmpeg_stream()
                        
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Streaming worker error: {e}')
                time.sleep(0.1)
    
    def image_callback(self, msg: Image):
        """Process incoming camera frames"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Resize if needed
            if cv_image.shape[1] != self.width or cv_image.shape[0] != self.height:
                cv_image = cv2.resize(cv_image, (self.width, self.height))
            
            # Add frame to streaming queue (non-blocking)
            try:
                self.frame_queue.put_nowait(cv_image.copy())
            except queue.Full:
                # Drop frame if queue is full
                pass
            
            # Handle recording
            if self.recording:
                self._record_frame(cv_image)
                
                # Check max duration
                if time.time() - self.recording_start_time > self.max_clip_duration:
                    self.get_logger().warn(f'Max clip duration reached, stopping recording')
                    self._stop_recording()
                    
        except Exception as e:
            self.get_logger().error(f'Image callback error: {e}')
    
    def _record_frame(self, frame):
        """Write frame to video file"""
        if self.video_writer is not None:
            try:
                self.video_writer.write(frame)
            except Exception as e:
                self.get_logger().error(f'Failed to write frame: {e}')
    
    def start_recording_callback(self, request, response):
        """Start recording video clip"""
        if self.recording:
            response.success = False
            response.message = 'Already recording'
            return response
        
        try:
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f'robonurse_clip_{timestamp}.mp4'
            self.current_recording_path = os.path.join(self.recordings_dir, filename)
            
            # Create VideoWriter
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self.video_writer = cv2.VideoWriter(
                self.current_recording_path,
                fourcc,
                self.fps,
                (self.width, self.height)
            )
            
            if not self.video_writer.isOpened():
                raise RuntimeError('Failed to open VideoWriter')
            
            self.recording = True
            self.recording_start_time = time.time()
            
            response.success = True
            response.message = f'Recording started: {filename}'
            self.get_logger().info(response.message)
            
        except Exception as e:
            response.success = False
            response.message = f'Failed to start recording: {e}'
            self.get_logger().error(response.message)
        
        return response
    
    def stop_recording_callback(self, request, response):
        """Stop recording video clip"""
        if not self.recording:
            response.success = False
            response.message = 'Not currently recording'
            return response
        
        self._stop_recording()
        
        response.success = True
        response.message = f'Recording stopped: {os.path.basename(self.current_recording_path)}'
        self.get_logger().info(response.message)
        
        return response
    
    def _stop_recording(self):
        """Internal method to stop recording"""
        if self.video_writer is not None:
            self.video_writer.release()
            self.video_writer = None
        
        self.recording = False
        self.recording_start_time = None
        
        # Log file size
        if self.current_recording_path and os.path.exists(self.current_recording_path):
            size_mb = os.path.getsize(self.current_recording_path) / (1024 * 1024)
            self.get_logger().info(f'Recording saved: {size_mb:.2f} MB')
    
    def _start_webrtc_server(self):
        """Start simple WebRTC signaling server (placeholder)"""
        self.get_logger().info(f'WebRTC signaling server on port {self.webrtc_port}')
        # TODO: Implement WebRTC signaling using aiortc or similar
        # For now, we rely on RTSP which is simpler and lower latency
    
    def destroy_node(self):
        """Cleanup on shutdown"""
        self.streaming = False
        
        if self.recording:
            self._stop_recording()
        
        if self.ffmpeg_process:
            try:
                self.ffmpeg_process.stdin.close()
                self.ffmpeg_process.terminate()
                self.ffmpeg_process.wait(timeout=3)
            except Exception:
                self.ffmpeg_process.kill()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VideoStreamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down video_stream_node')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()