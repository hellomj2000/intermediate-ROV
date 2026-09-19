#include <Servo.h>

#define NUM_CHANNELS 7
#define PWM_MIN 1000
#define PWM_MAX 2000
#define PWM_STOP 1500
#define RESET_INTERVAL 20
#define RESET_STEPS 100

const int pins[NUM_CHANNELS] = {2, 3, 4, 5, 6, 7, 8};
Servo channels[NUM_CHANNELS];
int currentPWM[NUM_CHANNELS];
int targetPWM[NUM_CHANNELS];

bool resetting = false;
int resetStep = 0;
unsigned long lastResetTime = 0;

String inputBuffer;

void setup() {
  Serial1.begin(9600);

  for (int i = 0; i < NUM_CHANNELS; i++) {
    channels[i].attach(pins[i]);
    channels[i].writeMicroseconds(PWM_STOP);
    currentPWM[i] = PWM_STOP;
    targetPWM[i] = PWM_STOP;
  }

  delay(2000);
}

void loop() {
  if (resetting) {
    doReset();
  }

  while (Serial1.available()) {
    char c = Serial1.read();
    if (c == '\n') {
      processCommand(inputBuffer);
      inputBuffer = "";
    } else if (c != '\r') {
      inputBuffer += c;
    }
  }
}

int clamp(int val) {
  if (val < PWM_MIN) return PWM_MIN;
  if (val > PWM_MAX) return PWM_MAX;
  return val;
}

void control(int ch, int pwm) {
  if (ch < 0 || ch >= NUM_CHANNELS) return;
  if (resetting) return;
  pwm = clamp(pwm);
  currentPWM[ch] = pwm;
  targetPWM[ch] = pwm;
  channels[ch].writeMicroseconds(pwm);
}

void allThrusters(int pwm) {
  pwm = clamp(pwm);
  for (int i = 0; i < NUM_CHANNELS; i++) {
    currentPWM[i] = pwm;
    targetPWM[i] = pwm;
    channels[i].writeMicroseconds(pwm);
  }
}

void startReset() {
  resetting = true;
  resetStep = 0;
  lastResetTime = 0;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    targetPWM[i] = PWM_STOP;
  }
}

void doReset() {
  unsigned long now = millis();
  if (now - lastResetTime < RESET_INTERVAL) return;
  lastResetTime = now;
  resetStep++;

  bool allDone = true;
  for (int i = 0; i < NUM_CHANNELS; i++) {
    int diff = targetPWM[i] - currentPWM[i];
    if (diff == 0) continue;
    allDone = false;

    int stepsLeft = RESET_STEPS - resetStep + 1;
    if (stepsLeft <= 0) {
      currentPWM[i] = targetPWM[i];
    } else {
      int step = diff / stepsLeft;
      if (step == 0) step = (diff > 0) ? 1 : -1;
      currentPWM[i] += step;
    }
    channels[i].writeMicroseconds(currentPWM[i]);
  }

  if (allDone || resetStep >= RESET_STEPS) {
    for (int i = 0; i < NUM_CHANNELS; i++) {
      currentPWM[i] = targetPWM[i];
      channels[i].writeMicroseconds(currentPWM[i]);
    }
    resetting = false;
  }
}

void printStatus() {
  Serial1.print("CH: ");
  for (int i = 0; i < NUM_CHANNELS; i++) {
    if (i > 0) Serial1.print(", ");
    Serial1.print(i);
    Serial1.print("=");
    Serial1.print(currentPWM[i]);
  }
  Serial1.print(" | reset=");
  Serial1.println(resetting ? "1" : "0");
}

void printHelp() {
  Serial1.println("ROV Debug Commands:");
  Serial1.println("  c <ch> <pwm>   Set channel (0-5=motor, 6=servo) to PWM 1000-2000");
  Serial1.println("  a <pwm>        All thrusters to same PWM");
  Serial1.println("  r              Reset all to 1500 (ramp over ~2s)");
  Serial1.println("  s              Print status");
  Serial1.println("  h              This help");
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char prefix = cmd.charAt(0);
  cmd = cmd.substring(1);
  cmd.trim();

  switch (prefix) {
    case 'c':
    case 'C': {
      int sp = cmd.indexOf(' ');
      if (sp < 0) return;
      int ch = cmd.substring(0, sp).toInt();
      int pwm = cmd.substring(sp + 1).toInt();
      control(ch, pwm);
      break;
    }
    case 'a':
    case 'A': {
      int pwm = cmd.toInt();
      allThrusters(pwm);
      break;
    }
    case 'r':
    case 'R':
      startReset();
      break;
    case 's':
    case 'S':
      printStatus();
      break;
    case 'h':
    case 'H':
      printHelp();
      break;
    default:
      Serial1.print("? ");
      Serial1.println(prefix);
      break;
  }
}
