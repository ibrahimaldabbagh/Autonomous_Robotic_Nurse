robonurse_hw

Includes:
- motor_control_node: subscribes to /cmd_vel and sends serial commands to Arduino Mega motor controller.
- wheel_odometry_node: reads odometry/status packets from Arduino and publishes /odom.
- Arduino firmware: mega_motor_control.ino implementing binary serial protocol, PWM control, encoder reading, sensors, and watchdog.

Instructions:
1. Flash the Arduino Mega with mega_motor_control.ino using Arduino IDE or PlatformIO.
2. Connect Arduino to Raspberry Pi via USB. Confirm /dev/ttyACM0 or update serial_port in config.
3. Build ROS2 package in workspace and run nodes.
