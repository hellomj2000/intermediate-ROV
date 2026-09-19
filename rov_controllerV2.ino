//safer and cleaner
//ignore some warnings if you are using vscode, it is false positive

//check the range and change to uint16_if need later
//btw internal arduino board will preprocess AX = a pin number 
//constexpr > #define b/c type safety and scope

constexpr uint8_t JOY1_X = A0;
constexpr uint8_t JOY1_Y = A1;
constexpr uint8_t JOY2_X = A2;
constexpr uint8_t JOY2_Y = A3;

constexpr uint32_t BAUD = 9600;
constexpr uint8_t DEADZONE = 15;
constexpr uint8_t LED_PIN = 13;

constexpr uint8_t PKT_START = 0xAA;
constexpr uint8_t PKT_END = 0x55;
constexpr uint8_t PKT_CMD_LEN = 7;
constexpr uint8_t PKT_TELEM_START = 0xBB;
constexpr uint8_t PKT_TELEM_LEN = 7;

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


  //telemetry logic code part, inversed and cleaned up
  // Todo: find a way to replace goto, works fine for now, but not good practice
  //wait for avalible full packet.
  if (Serial1.available() < PKT_TELEM_LEN) goto WaitAndExit;

    //check header
  if (Serial1.peek() != PKT_TELEM_START) {
    Serial1.read();
    goto WaitAndExit;
    }
    //drop header
    Serial1.read();
    //make sure all remaining bytes are read
    uint8_t tb[6];
    if (Serial1.readBytes(tb, 6) != 6) goto WaitAndExit;
    //verify ending
    if (tb[5] != PKT_END) goto WaitAndExit;
    //checksum for error detect
    if ((tb[0] ^ tb[1] ^ tb[2] ^ tb[3] ^ tb[4]) != 0) goto WaitAndExit;
    //guarded operation by logic, idk what is this tbh
    uint16_t batt = ((uint16_t)tb[0] << 8) | tb[1];
    bool isBattValid = (batt > 0 && batt < 10500);

    //this "? :" thingy is equal to if(isBattValid) digitalWrite(LED_PIN, HIGH); else digitalWrite(LED_PIN, LOW);
    digitalWrite(LED_PIN, isBattValid ? HIGH : LOW);

WaitAndExit:
    delay(10);
    return;
}

int8_t readAxis(uint8_t pin) {
  int raw = analogRead(pin);
  if (raw > 512 - DEADZONE && raw < 512 + DEADZONE) return 0;
  if (raw < 512) return map(raw, 0, 512 - DEADZONE, -127, 0);
  return map(raw, 512 + DEADZONE, 1023, 0, 127);
}
