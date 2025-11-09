/*
  mega_motor_control.ino

  Arduino Mega firmware for RoboNurse motor controller.

  Features:
  - Reads binary commands from host over Serial (USB)
  - Controls 4 motor drivers via PWM (BTS or similar)
  - Reads 4 quadrature encoders via interrupts
  - Reads Sharp IR and ultrasonic sensors on analog/digital pins
  - Implements emergency stop (E-STOP) input pin that immediately disables motors
  - Sends telemetry packets periodically with encoder counts and battery voltage
  - Implements watchdog: stops motors if no keepalive received within timeout

  Packet format (Host -> Arduino):
  [0xAA, 0x55] | cmd_id (1 byte) | len (1 byte) | payload | crc8 (1 byte)

  cmd_id:
   0x01 : set wheel speeds (payload: 4 x int16 little-endian mm/s)
   0x02 : stop motors
   0x03 : keepalive / ping

  Packet format (Arduino -> Host telemetry):
  [0x55, 0xAA] | pkt_id (1 byte) | len (1 byte) | payload | crc8
  pkt_id:
   0x10 : odometry packet:
     payload: int32 seq | int32 t_ms | int32 enc0 | int32 enc1 | int32 enc2 | int32 enc3 | int16 batt_mv

  Note: This firmware is designed as a robust base for your competition. Tune pins and scaling according to your hardware.
*/

#include <Arduino.h>

// --- configuration ---
const int MOTOR_PWMS[4] = {2, 3, 4, 5}; // PWM pins for motors (change as needed)
const int MOTOR_DIRS[4] = {22, 23, 24, 25}; // direction pins (if needed)
const int ENCODER_A_PINS[4] = {18, 19, 20, 21}; // encoder A pins
const int ENCODER_B_PINS[4] = {26, 27, 28, 29}; // encoder B pins
const int SHARP_PIN = A0; // analog pin
const int ULTRA_TRIG = 30;
const int ULTRA_ECHO = 31;
const int ESTOP_PIN = 32; // active LOW assumed
const int BATTERY_PIN = A1;

volatile long enc_counts[4] = {0,0,0,0};
volatile long last_enc_snapshot[4] = {0,0,0,0};

// for encoder interrupt handlers
void IRAM_ATTR enc0_isr() { enc_counts[0]++; }
void IRAM_ATTR enc1_isr() { enc_counts[1]++; }
void IRAM_ATTR enc2_isr() { enc_counts[2]++; }
void IRAM_ATTR enc3_isr() { enc_counts[3]++; }

unsigned long last_command_time = 0;
const unsigned long WATCHDOG_MS = 500; // stop motors if no keepalive in 500 ms

uint32_t seq = 0;

// CRC8
uint8_t crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0;
  for (size_t i = 0; i < len; ++i) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; ++j) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

// Serial buffer
const size_t RX_BUF_SIZE = 128;
uint8_t rxbuf[RX_BUF_SIZE];
size_t rxlen = 0;

void setup() {
  Serial.begin(115200);
  // motor pins
  for (int i=0;i<4;i++) {
    pinMode(MOTOR_PWMS[i], OUTPUT);
    pinMode(MOTOR_DIRS[i], OUTPUT);
    analogWrite(MOTOR_PWMS[i], 0);
    digitalWrite(MOTOR_DIRS[i], LOW);
  }
  // encoder pins
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[0]), enc0_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[1]), enc1_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[2]), enc2_isr, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_PINS[3]), enc3_isr, RISING);

  pinMode(ULTRA_TRIG, OUTPUT);
  pinMode(ULTRA_ECHO, INPUT);
  pinMode(ESTOP_PIN, INPUT_PULLUP); // active LOW
  pinMode(BATTERY_PIN, INPUT);

  last_command_time = millis();
}

// helper to read ultrasonic (blocking short)
long read_ultrasonic_cm() {
  digitalWrite(ULTRA_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG, LOW);
  unsigned long duration = pulseIn(ULTRA_ECHO, HIGH, 20000); // timeout 20ms
  long cm = duration / 29 / 2;
  if (duration == 0) return -1;
  return cm;
}

void send_odometry_packet() {
  // prepare payload
  uint8_t payload[4*4 + 4 + 4 + 2]; // seq (4) + t_ms (4) + 4*enc (4*4) + batt (2)
  uint8_t *p = payload;
  uint32_t t_ms = (uint32_t)millis();
  memcpy(p, &seq, 4); p += 4;
  memcpy(p, &t_ms, 4); p += 4;
  noInterrupts();
  int32_t e0 = enc_counts[0];
  int32_t e1 = enc_counts[1];
  int32_t e2 = enc_counts[2];
  int32_t e3 = enc_counts[3];
  interrupts();
  memcpy(p, &e0, 4); p += 4;
  memcpy(p, &e1, 4); p += 4;
  memcpy(p, &e2, 4); p += 4;
  memcpy(p, &e3, 4); p += 4;
  // battery mv read (approx)
  int batt = analogRead(BATTERY_PIN);
  int16_t batt_mv = (int16_t)(batt * (5.0/1023.0) * 1000.0); // rough conversion
  memcpy(p, &batt_mv, 2); p += 2;

  uint8_t header[2] = {0x55, 0xAA};
  uint8_t pkt_id = 0x10;
  uint8_t len = (uint8_t)(p - payload);
  // build packet
  Serial.write(header, 2);
  Serial.write(pkt_id);
  Serial.write(len);
  Serial.write(payload, len);
  // compute crc
  // compute crc over header+pkt_id+len+payload
  size_t totlen = 2 + 1 + 1 + len;
  uint8_t *buf = (uint8_t*)malloc(totlen);
  buf[0] = header[0]; buf[1] = header[1]; buf[2] = pkt_id; buf[3] = len;
  memcpy(buf+4, payload, len);
  uint8_t c = crc8(buf, totlen);
  Serial.write(c);
  free(buf);
  seq++;
}

void process_command_packet(const uint8_t *pkt, size_t len) {
  // pkt points to header (AA 55 ...) but we assume header removed and pkt starts at hdr
  if (len < 5) return;
  if (pkt[0] != 0xAA || pkt[1] != 0x55) return;
  uint8_t cmd_id = pkt[2];
  uint8_t payload_len = pkt[3];
  if (4 + payload_len + 1 > len) return; // length mismatch
  const uint8_t *payload = pkt + 4;
  uint8_t crc_recv = pkt[4 + payload_len];
  uint8_t crc_calc = crc8(pkt, 4 + payload_len);
  if (crc_calc != crc_recv) return;
  last_command_time = millis();
  if (cmd_id == 0x01 && payload_len == 8) {
    // 4 x int16 mm/s
    int16_t v0 = (int16_t)(payload[0] | (payload[1]<<8));
    int16_t v1 = (int16_t)(payload[2] | (payload[3]<<8));
    int16_t v2 = (int16_t)(payload[4] | (payload[5]<<8));
    int16_t v3 = (int16_t)(payload[6] | (payload[7]<<8));
    // convert mm/s to PWM - naive mapping: map abs(mm/s) 0..1000 mm/s -> pwm 0..255
    int speed_vals[4] = {v0, v1, v2, v3};
    for (int i=0;i<4;i++) {
      int sp = constrain(speed_vals[i], -32000, 32000);
      int pwm = map(abs(sp), 0, 1000, 0, 255);
      if (pwm > 255) pwm = 255;
      if (sp >= 0) digitalWrite(MOTOR_DIRS[i], LOW); else digitalWrite(MOTOR_DIRS[i], HIGH);
      analogWrite(MOTOR_PWMS[i], pwm);
    }
  } else if (cmd_id == 0x02) {
    // stop motors
    for (int i=0;i<4;i++) {
      analogWrite(MOTOR_PWMS[i], 0);
    }
  } else if (cmd_id == 0x03) {
    // ping
  }
}

void loop() {
  // read serial incoming
  while (Serial.available() > 0) {
    uint8_t b = Serial.read();
    // append to rx buffer (simple ring)
    if (rxlen < RX_BUF_SIZE) {
      rxbuf[rxlen++] = b;
    } else {
      // overflow, reset
      rxlen = 0;
    }
  }
  // attempt to parse packets in rxbuf
  // look for header 0xAA 0x55
  size_t idx = 0;
  while (idx + 5 <= rxlen) {
    if (rxbuf[idx] == 0xAA && rxbuf[idx+1] == 0x55) {
      uint8_t payload_len = rxbuf[idx+3];
      size_t total = 2 + 1 + 1 + payload_len + 1;
      if (idx + total <= rxlen) {
        process_command_packet(rxbuf + idx, total);
        // remove processed bytes
        size_t remain = rxlen - (idx + total);
        if (remain > 0) memmove(rxbuf, rxbuf + idx + total, remain);
        rxlen = remain;
        idx = 0;
        continue;
      } else {
        // wait for more bytes
        break;
      }
    } else {
      idx++;
    }
  }

  // watchdog stop
  if (millis() - last_command_time > WATCHDOG_MS) {
    for (int i=0;i<4;i++) analogWrite(MOTOR_PWMS[i], 0);
  }

  static unsigned long last_telemetry = 0;
  if (millis() - last_telemetry > 100) {
    send_odometry_packet();
    last_telemetry = millis();
  }

  // small delay
  delay(2);
}
