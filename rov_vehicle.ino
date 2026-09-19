#include <Servo.h>

#define BAUD 9600
#define NUM_MOTORS 6
#define SERVO_PIN 8
#define BATTERY_PIN A0

#define PKT_START 0xAA
#define PKT_END 0x55
#define PKT_TELEM_START 0xBB
#define PKT_CMD_LEN 7
#define PKT_TELEM_LEN 7

#define MAX_RAMP_STEP 4
#define PWM_MIN 1000
#define PWM_STOP 1500
#define PWM_MAX 2000

#define VOLTAGE_DIVIDER_RATIO 4.7f
#define VOLTAGE_REF 5.0f

enum ParserState { WAIT_START, READ_PAYLOAD, WAIT_END };

static const uint8_t motorPins[NUM_MOTORS] = {2, 3, 4, 5, 6, 7};
Servo motors[NUM_MOTORS];
Servo gripper;

uint8_t parserState = WAIT_START;
uint8_t payloadBuf[5];
uint8_t payloadIdx = 0;

int8_t cmdYaw = 0;
int8_t cmdSurge = 0;
uint8_t cmdGrip = 90;
int8_t cmdVertical = 0;
int currentMotors[6] = {0, 0, 0, 0, 0, 0};
bool cmdValid = false;

void setup() {
  Serial1.begin(BAUD);
  Serial.begin(BAUD);

  gripper.attach(SERVO_PIN);
  for (int i = 0; i < NUM_MOTORS; i++) {
    motors[i].attach(motorPins[i]);
    motors[i].writeMicroseconds(PWM_STOP);
  }
  delay(2000);
}

void loop() {
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    processByte(b);
  }

  if (cmdValid) {
    int targets[6];
    targets[0] = constrain(cmdSurge + cmdYaw, -127, 127);
    targets[1] = constrain(cmdSurge - cmdYaw, -127, 127);
    targets[2] = constrain(cmdSurge + cmdYaw, -127, 127);
    targets[3] = constrain(cmdSurge - cmdYaw, -127, 127);
    targets[4] = constrain(cmdVertical, -127, 127);
    targets[5] = constrain(cmdVertical, -127, 127);

    for (int i = 0; i < NUM_MOTORS; i++) {
      int diff = targets[i] - currentMotors[i];
      int step = constrain(abs(diff) / 2, 1, MAX_RAMP_STEP);
      if (diff > 0) currentMotors[i] = min(currentMotors[i] + step, targets[i]);
      else if (diff < 0) currentMotors[i] = max(currentMotors[i] - step, targets[i]);
      motors[i].writeMicroseconds(map(currentMotors[i], -127, 127, PWM_MIN, PWM_MAX));
    }
    gripper.write(cmdGrip);
  }

  sendTelemetry();
  delay(20);
}

void processByte(uint8_t b) {
  switch (parserState) {
    case WAIT_START:
      if (b == PKT_START) { parserState = READ_PAYLOAD; payloadIdx = 0; }
      break;
    case READ_PAYLOAD:
      payloadBuf[payloadIdx++] = b;
      if (payloadIdx == 5) parserState = WAIT_END;
      break;
    case WAIT_END:
      if (b == PKT_END) {
        if ((payloadBuf[0] ^ payloadBuf[1] ^ payloadBuf[2] ^ payloadBuf[3]) == payloadBuf[4]) {
          cmdYaw = (int8_t)payloadBuf[0];
          cmdSurge = (int8_t)payloadBuf[1];
          cmdGrip = payloadBuf[2];
          cmdVertical = (int8_t)payloadBuf[3];
          cmdValid = true;
        }
      }
      parserState = WAIT_START;
      break;
  }
}

void sendTelemetry() {
  static unsigned long lastSend = 0;
  if (millis() - lastSend < 100) return;
  lastSend = millis();

  int raw = analogRead(BATTERY_PIN);
  uint16_t battery = (uint16_t)(raw * (VOLTAGE_REF / 1023.0f) * VOLTAGE_DIVIDER_RATIO * 1000.0f);

  uint8_t buf[PKT_TELEM_LEN];
  buf[0] = PKT_TELEM_START;
  buf[1] = (uint8_t)(battery >> 8);
  buf[2] = (uint8_t)(battery & 0xFF);
  buf[3] = 0;
  buf[4] = 0;
  buf[5] = buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
  buf[6] = PKT_END;
  Serial1.write(buf, PKT_TELEM_LEN);
}
