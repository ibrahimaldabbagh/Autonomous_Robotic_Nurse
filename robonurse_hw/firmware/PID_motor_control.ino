// ====================================================================
// mega_motor_control.ino - Arduino Mega Firmware
// Low-level motor control, sensor reading, and serial communication
// Uses rosserial-like protocol over standard Serial
// ====================================================================

#include <PID_v1.h> 

// --- Pin Definitions ---
// Motor Control (BTS7960/PWM) - Example Pins, adjust for your BTS driver
#define FL_PWM_PIN 2   // Left PWM for Motor 1
#define FR_PWM_PIN 3   // Left PWM for Motor 2
#define RL_PWM_PIN 4   // Left PWM for Motor 3
#define RR_PWM_PIN 5   // Left PWM for Motor 4

// Encoder Pins (Interrupts 2, 3, 18, 19, 20, 21 on Mega) - Example
#define FL_ENC_A 18    // Interrupt Pin (INT5)
#define FR_ENC_A 19    // Interrupt Pin (INT4)
// Use custom port reads for the others if needed, or non-INT pins for simple count

// Sensor Pins
#define E_STOP_PIN 53  // Physical E-Stop button (Active LOW/Pullup)
#define BUMP_FL_PIN 52 // Bump sensor Front-Left
#define IR_FRONT_PIN A0 // Analog Read for Sharp IR
#define ULTRA_TRIG 30
#define ULTRA_ECHO 31

// Actuator Pins (Example for dispensing mechanism)
#define STEPPER_PULSE 40
#define STEPPER_DIR 41
#define SERVO_PIN 42
#define LIMIT_SWITCH_DISPENSER 43

// --- ROS-like Serial Protocol Commands ---
#define CMD_VEL 0x01
#define MOTOR_STATUS 0x02
#define ACTUATOR_CMD 0x03
#define ACTUATOR_ACK 0x04

// --- Motor/Kinematics Constants ---
const float WHEEL_RADIUS = 0.05; // 5cm
const float BASE_RADIUS = 0.25; // Distance from center to wheel
const float MAX_VEL = 1.0; // Max m/s
const int MAX_PWM = 255; 

// --- Global State ---
volatile long fl_enc_count = 0;
volatile long fr_enc_count = 0;
volatile long rl_enc_count = 0;
volatile long rr_enc_count = 0;

float current_linear_x = 0.0;
float current_linear_y = 0.0;
float current_angular_z = 0.0;

unsigned long last_cmd_time = 0;
const unsigned int WATCHDOG_TIMEOUT = 500; // 500ms

bool emergency_stop_active = false;

// --- PID Setup (Simplified Velocity Control Example) ---
// PID Input/Output/Setpoint for a single wheel (repeat for all four)
double Setpoint_fl, Input_fl, Output_fl;
PID fl_PID(&Input_fl, &Output_fl, &Setpoint_fl, 2.0, 0.1, 0.0, DIRECT); // Kp, Ki, Kd

// --- Function Prototypes ---
void processSerialCommand();
void sendMotorStatus();
void mecanumInverseKinematics(float vx, float vy, float wz, int& pwm_fl, int& pwm_fr, int& pwm_rl, int& pwm_rr);
void readSensors();
void handleActuatorCommand(uint8_t item_id, int32_t slot_id);
// Encoder Interrupts (Simplified, assumes basic count)
void updateEncoderFL() { fl_enc_count++; }
void updateEncoderFR() { fr_enc_count++; }


void setup() {
  Serial.begin(115200); // Communication with Raspberry Pi
  
  // Pin Setup
  pinMode(E_STOP_PIN, INPUT_PULLUP);
  pinMode(BUMP_FL_PIN, INPUT_PULLUP);
  pinMode(IR_FRONT_PIN, INPUT);
  // Motor PWM pins set to output
  pinMode(FL_PWM_PIN, OUTPUT);
  // ... (setup other motor and actuator pins)

  // Encoder Interrupts
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), updateEncoderFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), updateEncoderFR, CHANGE);
  // Note: Mega has only 6 external interrupts, more encoders need Pin Change Interrupts or polling.

  // PID Initialization (Simplified - must tune these aggressively)
  Setpoint_fl = 0; // Target velocity
  fl_PID.SetMode(AUTOMATIC);
  fl_PID.SetOutputLimits(-MAX_PWM, MAX_PWM);
  // Repeat for FR, RL, RR
  
  // Watchdog initialization
  last_cmd_time = millis();
}

void loop() {
  // 1. Process incoming Serial commands (non-blocking)
  processSerialCommand();

  // 2. Check Watchdog & E-Stop
  emergency_stop_active = (digitalRead(E_STOP_PIN) == LOW) || (millis() - last_cmd_time > WATCHDOG_TIMEOUT);

  if (emergency_stop_active) {
    // Cut power immediately to all motors
    analogWrite(FL_PWM_PIN, 0);
    // ... all other motors to 0
    current_linear_x = 0; current_linear_y = 0; current_angular_z = 0;
  } else {
    // 3. Sensor/Odometry Update (Simplified - read encoder counts)
    // In a full implementation, this is where you read velocity from encoders 
    // to feed into the PID Input.
    
    // 4. Run Motor Control (PID)
    // Input_fl = getWheelVelocityFL(); // Get actual velocity from encoder
    // Setpoint_fl = current_target_velocity_fl; // Target from Inverse Kinematics
    // fl_PID.Compute(); 
    // analogWrite(FL_PWM_PIN, (int)Output_fl); 
    
    // --- Simplified Control (Direct PWM mapping) ---
    // Calculate required PWM values based on the last valid CMD_VEL
    int pwm_fl, pwm_fr, pwm_rl, pwm_rr;
    mecanumInverseKinematics(current_linear_x, current_linear_y, current_angular_z, pwm_fl, pwm_fr, pwm_rl, pwm_rr);

    analogWrite(FL_PWM_PIN, abs(pwm_fl));
    // ... analogWrite other motors
    // Handle Direction Pins based on sign of pwm_fl etc.
  }
  
  // 5. Read Analog/Digital Sensors (IR, Ultra, Bump)
  readSensors();

  // 6. Send Status Feedback (Slower rate than control loop)
  static unsigned long last_status_send = 0;
  if (millis() - last_status_send > 50) { // 20Hz update to ROS
    sendMotorStatus();
    last_status_send = millis();
  }
}


// ====================================================================
// Serial Protocol Implementation
// ====================================================================

void processSerialCommand() {
  while (Serial.available() >= 6) { // Minimum packet size (Header + Cmd + Payload)
    if (Serial.read() == 0xFF) { // Check Header Byte (Simple 0xFF header)
      uint8_t cmd_id = Serial.read();
      
      if (cmd_id == CMD_VEL && Serial.available() >= 12) { // CMD_VEL: 3x float32 = 12 bytes
        // Read linear_x
        Serial.readBytes((char*)&current_linear_x, 4);
        // Read linear_y
        Serial.readBytes((char*)&current_linear_y, 4);
        // Read angular_z
        Serial.readBytes((char*)&current_angular_z, 4);

        last_cmd_time = millis();
        // Optional: Send a low-level ACK
      } 
      else if (cmd_id == ACTUATOR_CMD && Serial.available() >= 5) { // ACTUATOR_CMD: uint8 + int32 = 5 bytes
        uint8_t item_id;
        int32_t slot_id;
        Serial.readBytes((char*)&item_id, 1);
        Serial.readBytes((char*)&slot_id, 4);
        
        handleActuatorCommand(item_id, slot_id);
      }
      else {
        // Bad packet - discard the rest of the buffer up to next header or timeout
        while(Serial.available() > 0 && Serial.peek() != 0xFF) Serial.read();
      }
    }
  }
}

void sendMotorStatus() {
  uint8_t header = 0xFF;
  uint8_t cmd_id = MOTOR_STATUS;
  
  // Start packet (Header + Command ID)
  Serial.write(header);
  Serial.write(cmd_id);
  
  // Payload: MotorStatus fields (Must match definition exactly!)
  // 1. Encoder Counts (4x int32)
  Serial.write((byte*)&fl_enc_count, 4);
  Serial.write((byte*)&fr_enc_count, 4);
  Serial.write((byte*)&rl_enc_count, 4);
  Serial.write((byte*)&rr_enc_count, 4);
  
  // 2. Motor Currents (4x float32 - placeholder/dummy value)
  float current = 0.5; // Replace with ADC read
  Serial.write((byte*)&current, 4); Serial.write((byte*)&current, 4); 
  Serial.write((byte*)&current, 4); Serial.write((byte*)&current, 4);
  
  // 3. Battery Voltage (float32 - placeholder/dummy value)
  float voltage = 12.5; 
  Serial.write((byte*)&voltage, 4);
  
  // 4. E-Stop Status (bool)
  Serial.write((byte*)&emergency_stop_active, 1);

  // Note: No CRC/Checksum for simplicity, but HIGHLY recommended for robustness
}


// ====================================================================
// Kinematics & Low-Level Control
// ====================================================================

void mecanumInverseKinematics(float vx, float vy, float wz, int& pwm_fl, int& pwm_fr, int& pwm_rl, int& pwm_rr) {
  // v_wheel = v_x + v_y * sign(wheel_angle) + R * omega
  // For standard X-pattern mecanum:
  // FL (+x, -y, +z)
  // FR (+x, +y, -z)
  // RL (+x, +y, +z)
  // RR (+x, -y, -z)

  float v_fl = vx - vy - BASE_RADIUS * wz; 
  float v_fr = vx + vy + BASE_RADIUS * wz; 
  float v_rl = vx + vy - BASE_RADIUS * wz; 
  float v_rr = vx - vy + BASE_RADIUS * wz;
  
  // Convert velocity (m/s) to PWM (0-255). Simple linear scaling.
  pwm_fl = (int)((v_fl / MAX_VEL) * MAX_PWM);
  pwm_fr = (int)((v_fr / MAX_VEL) * MAX_PWM);
  pwm_rl = (int)((v_rl / MAX_VEL) * MAX_PWM);
  pwm_rr = (int)((v_rr / MAX_VEL) * MAX_PWM);
  
  // Clip to max PWM
  pwm_fl = constrain(pwm_fl, -MAX_PWM, MAX_PWM);
  pwm_fr = constrain(pwm_fr, -MAX_PWM, MAX_PWM);
  pwm_rl = constrain(pwm_rl, -MAX_PWM, MAX_PWM);
  pwm_rr = constrain(pwm_rr, -MAX_PWM, MAX_PWM);
}


// ====================================================================
// Sensor & Actuator Implementation
// ====================================================================

void readSensors() {
  // Read Sharp IR: Simple Analog to distance (calibration needed)
  int ir_val = analogRead(IR_FRONT_PIN);
  // (Conversion to distance happens in the ROS node or here if calibrated)

  // Read Ultrasonic: Standard PulseIn logic
  // (Implement the ultrasonic reading logic here)
  
  // Read Bump Sensors
  bool bump_fl = (digitalRead(BUMP_FL_PIN) == LOW);
  // If any critical sensor detects collision, you might trigger E-stop here:
  // if (bump_fl) emergency_stop_active = true;
}

void handleActuatorCommand(uint8_t item_id, int32_t slot_id) {
  // This is where you implement the servo/stepper control sequence
  // to dispense the requested item from the specific slot.
  bool success = false;
  
  if (item_id == 1) { // Medicine
    // Example: Stepper move
    for(int i = 0; i < 200; i++) {
      digitalWrite(STEPPER_PULSE, HIGH); delay(2);
      digitalWrite(STEPPER_PULSE, LOW); delay(2);
    }
    // Check limit switch/photo sensor for successful dispense
    if (digitalRead(LIMIT_SWITCH_DISPENSER) == HIGH) {
      success = true;
    }
  } else if (item_id == 2) { // Drink
    // Example: Servo motion
    // ...
    success = true; // Assume success for this example
  }

  // Send ACK back to Raspberry Pi
  uint8_t header = 0xFF;
  uint8_t cmd_id = ACTUATOR_ACK;
  Serial.write(header);
  Serial.write(cmd_id);
  Serial.write((byte*)&success, 1);
  Serial.write((byte*)&item_id, 1);
}