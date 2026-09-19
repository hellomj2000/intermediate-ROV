#define JOY1_X A0
#define JOY1_Y A1
#define JOY2_X A2
#define JOY2_Y A3

#define BAUD 9600
#define DEADZONE 15
#define LED_PIN 13

#define PKT_START 0xAA
#define PKT_END 0x55
#define PKT_CMD_LEN 7
#define PKT_TELEM_START 0xBB
#define PKT_TELEM_LEN 7

void setup() {
  Serial1.begin(BAUD);
  Serial.begin(BAUD);
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  int8_t yaw = readAxis(JOY1_X);
  int8_t surge = readAxis(JOY1_Y);
  uint8_t grip = map(analogRead(JOY2_X), 0, 1023, 0, 180);
  int8_t vertical = readAxis(JOY2_Y);

  uint8_t buf[PKT_CMD_LEN];
  buf[0] = PKT_START;
  buf[1] = (uint8_t)yaw;
  buf[2] = (uint8_t)surge;
  buf[3] = grip;
  buf[4] = (uint8_t)vertical;
  buf[5] = buf[1] ^ buf[2] ^ buf[3] ^ buf[4];
  buf[6] = PKT_END;
  Serial1.write(buf, PKT_CMD_LEN);

  if (Serial1.available() >= PKT_TELEM_LEN) {
    if (Serial1.peek() == PKT_TELEM_START) {
      Serial1.read();
      uint8_t tb[6];
      if (Serial1.readBytes(tb, 6) == 6 && tb[5] == PKT_END) {
        if ((tb[0] ^ tb[1] ^ tb[2] ^ tb[3] ^ tb[4]) == 0) {
          uint16_t batt = ((uint16_t)tb[0] << 8) | tb[1];
          if (batt > 0 && batt < 10500) digitalWrite(LED_PIN, HIGH);
          else digitalWrite(LED_PIN, LOW);
        }
      }
    } else {
      Serial1.read();
    }
  }

  delay(10);
}

int8_t readAxis(uint8_t pin) {
  int raw = analogRead(pin);
  if (raw > 512 - DEADZONE && raw < 512 + DEADZONE) return 0;
  if (raw < 512) return map(raw, 0, 512 - DEADZONE, -127, 0);
  return map(raw, 512 + DEADZONE, 1023, 0, 127);
}
