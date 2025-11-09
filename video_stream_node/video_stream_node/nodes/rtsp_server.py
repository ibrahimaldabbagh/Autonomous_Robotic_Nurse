"""rtsp_server.py

Simple RTSP server using GStreamer for receiving H.264 stream.
This is a helper script to run an RTSP server that the video_stream_node can stream to.

Usage:
  python rtsp_server.py --port 8554 --path /robonurse

Requirements:
  - GStreamer with RTSP server plugin
  - sudo apt-get install gstreamer1.0-rtsp

Author: Ibrahim Aldabbagh
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import argparse


class RTSPServer:
    def __init__(self, port=8554, path='/robonurse'):
        Gst.init(None)
        
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(port))
        
        factory = GstRtspServer.RTSPMediaFactory()
        
        # GStreamer pipeline to receive and serve H.264
        launch_str = (
            "( "
            "udpsrc port=5000 ! "
            "application/x-rtp,media=video,encoding-name=H264,payload=96 ! "
            "rtph264depay ! "
            "rtph264pay name=pay0 pt=96 "
            ")"
        )
        
        factory.set_launch(launch_str)
        factory.set_shared(True)
        
        mounts = self.server.get_mount_points()
        mounts.add_factory(path, factory)
        
        self.server.attach(None)
        
        print(f"RTSP server started on rtsp://0.0.0.0:{port}{path}")
    
    def run(self):
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            print("\\nShutting down RTSP server")


def main():
    parser = argparse.ArgumentParser(description='Simple RTSP Server for RoboNurse')
    parser.add_argument('--port', type=int, default=8554, help='RTSP port')
    parser.add_argument('--path', type=str, default='/robonurse', help='RTSP path')
    args = parser.parse_args()
    
    server = RTSPServer(port=args.port, path=args.path)
    server.run()


if __name__ == '__main__':
    main()`,

    'video_stream_node/config/video_stream_params.yaml': `video_stream_node:
  ros__parameters:
    # RTSP streaming configuration
    stream_url: "rtsp://0.0.0.0:8554/robonurse"
    
    # WebRTC configuration (experimental)
    enable_webrtc: false
    webrtc_port: 8080
    
    # Video encoding parameters
    codec: "h264"              # h264 or h265
    bitrate_kbps: 2000         # Target bitrate in kbps
    fps: 15                    # Stream frame rate
    width: 640                 # Stream width
    height: 480                # Stream height
    
    # Recording parameters
    recordings_dir: "/tmp/robonurse_recordings"
    max_clip_duration_sec: 60.0  # Maximum clip length in seconds
    
    # Buffer configuration
    buffer_size: 30            # Frame buffer size (frames)`,

    'video_stream_node/launch/video_stream.launch.py': `#!/usr/bin/env python3
"""
Launch file for video_stream_node
Author: Ibrahim Aldabbagh
Email: eng.ibrahim.aldabbagh@gmail.com
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('video_stream_node')
    config_file = os.path.join(pkg_dir, 'config', 'video_stream_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Video stream node
    video_stream_node = Node(
        package='video_stream_node',
        executable='video_stream_node',
        name='video_stream_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        video_stream_node
    ])