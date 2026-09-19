// Arduino 1 (Surface) - 2 joysticks → serial → Arduino 2
//   A0 (Left X = Yaw),    A1 (Left Y = Surge)
//   A2 (Right X = Sway),  A3 (Right Y = Servo)
//
// Serial: TX1(18) → Sub RX1,  RX1(19) ← Sub TX1,  GND ↔ GND

#define BAUD 57600
#define RR 3

#define JOY_LX A0
#define JOY_LY A1
#define JOY_RX A2
#define JOY_RY A3

const int CENTER = 512;
const int DEADBAND = 20;

byte packetCounter = 0;

int flx = 512, fly = 512, frx = 512, fry = 512;

byte outSurge = 127, outSway = 127, outYaw = 127;

void setup() {
  Serial1.begin(BAUD);
}

void loop() {
  int lx = analogRead(JOY_LX);
  int ly = analogRead(JOY_LY);
  int rx = analogRead(JOY_RX);
  int ry = analogRead(JOY_RY);

  flx = (flx * 15 + lx) >> 4;
  fly = (fly * 15 + ly) >> 4;
  frx = (frx * 15 + rx) >> 4;
  fry = (fry * 15 + ry) >> 4;

  byte tgtSurge = mapAxis(fly);
  byte tgtYaw   = mapAxis(flx);
  byte tgtSway  = mapAxis(frx);
  byte svVal    = mapAxis(fry);

  outSurge = rampTo(outSurge, tgtSurge);
  outSway  = rampTo(outSway,  tgtSway);
  outYaw   = rampTo(outYaw,   tgtYaw);

  byte packet[9];
  packet[0] = 0xAA;
  packet[1] = outSurge;
  packet[2] = outSway;
  packet[3] = outYaw;
  packet[4] = 127;
  packet[5] = svVal;
  packet[6] = 127;
  packet[7] = packetCounter++;

  byte xorSum = 0;
  for (int i = 0; i < 8; i++) xorSum ^= packet[i];
  packet[8] = xorSum;

  Serial1.write(packet, 9);

  delay(20);
}

byte mapAxis(int raw) {
  int centered = raw - CENTER;
  if (abs(centered) < DEADBAND) return 127;
  float val = (float)centered / (CENTER - DEADBAND);
  val = constrain(val, -1.0, 1.0);
  return (byte)(val * 127.0 + 127.5);
}

byte rampTo(byte cur, byte tgt) {
  if (cur < tgt) {
    int next = cur + RR;
    return next > tgt ? tgt : (byte)next;
  }
  if (cur > tgt) {
    int next = cur - RR;
    return next < tgt ? tgt : (byte)next;
  }
  return cur;
}
