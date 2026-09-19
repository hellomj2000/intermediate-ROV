// Arduino 2 (Sub) - receives serial, drives 6 thrusters + 1 servo
//
// Wiring:
//   Serial: RX1(19) ← Shore TX1,  TX1(18) → Shore RX1,  GND ↔ GND
//   ESCs:  pins 2,3,4,5,6,7
//   Servo: pin 8
//
// Packet: [0xAA] [SURGE] [SWAY] [YAW] [---] [SERVO] [AUX] [COUNT] [XOR]

#include <Servo.h>

#define BAUD 57600
#define PWM_MIN 1100
#define PWM_MAX 1900
#define PWM_CENTER 1500
#define TIMEOUT_MS 500
#define PACKET_LEN 9

const int THRUSTER_PINS[6] = {2, 3, 4, 5, 6, 7};
const int SERVO_PIN = 8;

Servo thrusters[6];
Servo servo;

byte buf[PACKET_LEN];
int idx = 0;
unsigned long lastPacketTime = 0;

void setup() {
  Serial1.begin(BAUD);

  for (int i = 0; i < 6; i++) {
    thrusters[i].attach(THRUSTER_PINS[i]);
    thrusters[i].writeMicroseconds(PWM_CENTER);
  }

  servo.attach(SERVO_PIN);
  servo.writeMicroseconds(1500);

  lastPacketTime = millis();
  delay(2000);
}

void loop() {
  while (Serial1.available()) {
    byte b = Serial1.read();

    if (idx == 0 && b != 0xAA) continue;

    buf[idx++] = b;

    if (idx == PACKET_LEN) {
      byte xorSum = 0;
      for (int i = 0; i < PACKET_LEN - 1; i++) xorSum ^= buf[i];

      if (xorSum == buf[PACKET_LEN - 1]) {
        processPacket(buf);
        lastPacketTime = millis();
      }
      idx = 0;
    }
  }

  if (millis() - lastPacketTime > TIMEOUT_MS) {
    for (int i = 0; i < 6; i++) {
      thrusters[i].writeMicroseconds(PWM_CENTER);
    }
    servo.writeMicroseconds(1500);
  }
}

void processPacket(byte* pkt) {
  float surge = constrain((pkt[1] - 127.0) / 127.0, -1.0, 1.0);
  float sway  = constrain((pkt[2] - 127.0) / 127.0, -1.0, 1.0);
  float yaw   = constrain((pkt[3] - 127.0) / 127.0, -1.0, 1.0);

  float t[6];
  t[0] =  surge * 0.707 + sway * 0.707 + yaw * 0.707;
  t[1] =  surge * 0.707 - sway * 0.707 - yaw * 0.707;
  t[2] =  surge * 0.707 - sway * 0.707 + yaw * 0.707;
  t[3] =  surge * 0.707 + sway * 0.707 - yaw * 0.707;
  t[4] =  0.0;
  t[5] =  0.0;

  float maxVal = 0.0;
  for (int i = 0; i < 6; i++) {
    float absVal = t[i] > 0 ? t[i] : -t[i];
    if (absVal > maxVal) maxVal = absVal;
  }
  if (maxVal > 1.0) {
    for (int i = 0; i < 6; i++) t[i] /= maxVal;
  }

  for (int i = 0; i < 6; i++) {
    int pwm = constrain(
      PWM_CENTER + (int)(t[i] * (PWM_MAX - PWM_CENTER)),
      PWM_MIN, PWM_MAX
    );
    thrusters[i].writeMicroseconds(pwm);
  }

  int svPWM = map(pkt[5], 0, 255, 600, 2400);
  servo.writeMicroseconds(svPWM);
}
