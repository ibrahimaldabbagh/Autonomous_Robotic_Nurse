
**Author:** Ibrahim Aldabbagh  
**Email:** eng.ibrahim.aldabbagh@gmail.com  
**Version:** 1.0.0

## Overview

The \`video_stream_node\` provides real-time video streaming capabilities for the RoboNurse operator interface. It supports:

- **RTSP Streaming**: Low-latency H.264 streaming for operator monitoring
- **WebRTC Support**: Browser-based streaming (experimental)
- **Video Recording**: On-demand clip recording via ROS services
- **Multiple Codecs**: H.264 and H.265 support

## Features

- ✅ Real-time H.264/H.265 encoding
- ✅ RTSP streaming with FFmpeg
- ✅ Configurable bitrate and resolution
- ✅ Recording service for incident clips
- ✅ Automatic reconnection on stream failure
- ✅ Frame dropping to prevent buffer overflow

## Dependencies

### System Packages
\`\`\`bash
sudo apt-get update
sudo apt-get install -y \\
    ffmpeg \\
    libavcodec-dev \\
    libavformat-dev \\
    libavutil-dev \\
    libswscale-dev \\
    gstreamer1.0-tools \\
    gstreamer1.0-plugins-base \\
    gstreamer1.0-plugins-good \\
    gstreamer1.0-plugins-bad \\
    gstreamer1.0-rtsp
\`\`\`

### Python Packages
\`\`\`bash
pip3 install opencv-python opencv-contrib-python
\`\`\`

## Installation

1. **Clone into your ROS 2 workspace:**
\`\`\`bash
cd ~/ros2_ws/src
cp -r video_stream_node .
\`\`\`

2. **Build the package:**
\`\`\`bash
cd ~/ros2_ws
colcon build --packages-select video_stream_node
source install/setup.bash
\`\`\`

## Configuration

Edit \`config/video_stream_params.yaml\`:

\`\`\`yaml
video_stream_node:
  ros__parameters:
    stream_url: "rtsp://0.0.0.0:8554/robonurse"
    codec: "h264"
    bitrate_kbps: 2000
    fps: 15
    width: 640
    height: 480
    recordings_dir: "/tmp/robonurse_recordings"
\`\`\`

## Usage

### Running the Node

**Option 1: Using launch file**
\`\`\`bash
ros2 launch video_stream_node video_stream.launch.py
\`\`\`

**Option 2: Direct execution**
\`\`\`bash
ros2 run video_stream_node video_stream_node --ros-args --params-file config/video_stream_params.yaml
\`\`\`

### Starting an RTSP Server

For testing, you can use the included RTSP server helper:

\`\`\`bash
python3 video_stream_node/rtsp_server.py --port 8554 --path /robonurse
\`\`\`

### Viewing the Stream

**Using VLC:**
\`\`\`bash
vlc rtsp://JETSON_IP:8554/robonurse
\`\`\`

**Using FFplay:**
\`\`\`bash
ffplay -rtsp_transport tcp rtsp://JETSON_IP:8554/robonurse
\`\`\`

**Using GStreamer:**
\`\`\`bash
gst-launch-1.0 rtspsrc location=rtsp://JETSON_IP:8554/robonurse ! decodebin ! autovideosink
\`\`\`

## Services

### Start Recording
\`\`\`bash
ros2 service call /video/start_recording std_srvs/srv/Trigger
\`\`\`

### Stop Recording
\`\`\`bash
ros2 service call /video/stop_recording std_srvs/srv/Trigger
\`\`\`

## Topics

### Subscribed Topics
- \`/camera/image_raw\` (sensor_msgs/Image): Raw camera feed from perception_camera_node

## Architecture

\`\`\`
┌─────────────────┐
│ Camera Node     │
│ (perception)    │
└────────┬────────┘
         │ /camera/image_raw
         │
         v
┌─────────────────┐
│ Video Stream    │
│ Node (Jetson)   │──────> RTSP Stream (rtsp://IP:8554/robonurse)
│                 │
│ - H.264 Encode  │──────> Recording (on demand)
│ - FFmpeg Stream │
└─────────────────┘
         │
         v
┌─────────────────┐
│ Operator HMI    │
│ (Tablet/PC)     │
└─────────────────┘
\`\`\`

## Performance Tuning

### For Jetson Nano:
- Use 480p @ 15fps for best balance
- Set bitrate to 1500-2000 kbps
- Enable hardware acceleration if available

### For Higher Performance:
\`\`\`yaml
width: 1280
height: 720
fps: 30
bitrate_kbps: 4000
\`\`\`

## Troubleshooting

### FFmpeg Not Starting
- Check if FFmpeg is installed: \`which ffmpeg\`
- Verify RTSP URL is reachable
- Check firewall settings

### High Latency
- Reduce resolution and FPS
- Lower bitrate
- Use \`ultrafast\` preset (already default)
- Ensure good network connection

### Recording Fails
- Check disk space
- Verify \`recordings_dir\` has write permissions:
  \`\`\`bash
  mkdir -p /tmp/robonurse_recordings
  chmod 777 /tmp/robonurse_recordings
  \`\`\`

### Stream Drops Frames
- Increase \`buffer_size\` parameter
- Reduce source camera FPS
- Lower encoding resolution

## Integration with Operator HMI

The operator Android/tablet app should connect to:
\`\`\`
rtsp://JETSON_IP:8554/robonurse
\`\`\`

Use libraries like:
- **Android**: ExoPlayer with RTSP support
- **Web**: video.js with RTSP/HLS plugin
- **Desktop**: VLC, FFmpeg, GStreamer

## Testing

### Test video stream is working:
\`\`\`bash
# Terminal 1: Run camera node
ros2 run perception_camera_node perception_camera_node

# Terminal 2: Run video stream node
ros2 run video_stream_node video_stream_node

# Terminal 3: View with VLC
vlc rtsp://localhost:8554/robonurse
\`\`\`

### Test recording:
\`\`\`bash
# Start recording
ros2 service call /video/start_recording std_srvs/srv/Trigger

# Wait 10 seconds...

# Stop recording
ros2 service call /video/stop_recording std_srvs/srv/Trigger

# Check recording
ls -lh /tmp/robonurse_recordings/
\`\`\`

## License

MIT License - Ibrahim Aldabbagh
