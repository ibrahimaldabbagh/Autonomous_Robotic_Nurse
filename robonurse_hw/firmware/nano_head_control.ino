// ====================================================================
// nano_head_control.ino - Arduino Nano Firmware (ROSSERIAL)
// Controls head servos and speaker playback 
// ====================================================================

#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
#include <DFRobotDFPlayerMini.h> // Library for MP3 module
#include <SoftwareSerial.h>

// --- Pin Definitions ---
#define HEAD_TILT_PIN 9
#define HEAD_PAN_PIN 10
#define DFPLAYER_RX_PIN 2 // Connect to MP3 TX
#define DFPLAYER_TX_PIN 3 // Connect to MP3 RX

// --- Hardware Objects ---
Servo pan_servo;
Servo tilt_servo;
SoftwareSerial mp3_serial(DFPLAYER_RX_PIN, DFPLAYER_TX_PIN);
DFRobotDFPlayerMini mp3_player;

// --- ROS Setup ---
ros::NodeHandle nh;

// Publishers
std_msgs::Int16 pan_status_msg;
ros::Publisher pan_status_pub("head/pan_status", &pan_status_msg);

// Callbacks for Subscriptions
void pan_cb(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  pan_servo.write(angle);
  pan_status_msg.data = angle; // Update status immediately
}
void tilt_cb(const std_msgs::Int16& msg) {
  int angle = constrain(msg.data, 0, 180);
  tilt_servo.write(angle);
}
void sound_cb(const std_msgs::Int16& msg) {
  // Play sound file corresponding to the index
  if (mp3_player.available()) {
    mp3_player.play(msg.data); // Assumes MP3 files are named 0001.mp3, 0002.mp3, etc.
  }
}

// Subscribers (Topics MUST match the head_interact_bridge node)
ros::Subscriber<std_msgs::Int16> sub_pan("head/pan_command", &pan_cb);
ros::Subscriber<std_msgs::Int16> sub_tilt("head/tilt_command", &tilt_cb);
ros::Subscriber<std_msgs::Int16> sub_sound("head/sound_command", &sound_cb);


void setup() {
  // --- Servo Setup ---
  pan_servo.attach(HEAD_PAN_PIN);
  tilt_servo.attach(HEAD_TILT_PIN);
  pan_servo.write(90); // Neutral position
  tilt_servo.write(90);
  
  // --- MP3 Player Setup ---
  mp3_serial.begin(9600);
  if (mp3_player.begin(mp3_serial)) {
    mp3_player.volume(20); 
    mp3_player.play(1); // Play a friendly startup sound (0001.mp3)
  }
  
  // --- ROS Setup ---
  nh.initNode();
  nh.subscribe(sub_pan);
  nh.subscribe(sub_tilt);
  nh.subscribe(sub_sound);
  nh.advertise(pan_status_pub);
}

void loop() {
  nh.spinOnce();
  
  // Publish current pan position periodically
  static unsigned long last_pub_time = 0;
  if (millis() - last_pub_time > 100) { // 10Hz
    pan_status_pub.publish(&pan_status_msg);
    last_pub_time = millis();
  }
  
  delay(1); 
}