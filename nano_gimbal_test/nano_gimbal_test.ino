#include <Servo.h>

const byte YAW_PIN = 3;
const byte PITCH_PIN = 5;

const int YAW_CENTER = 90;
const int PITCH_CENTER = 90;

// First keep a conservative range. Increase only after checking the mechanism.
const int YAW_MIN = 30;
const int YAW_MAX = 150;
const int PITCH_MIN = 45;
const int PITCH_MAX = 135;

const bool YAW_REVERSE = false;
const bool PITCH_REVERSE = false;

Servo yawServo;
Servo pitchServo;

int yawAngle = YAW_CENTER;
int pitchAngle = PITCH_CENTER;

int mapServoAngle(int angle, bool reverseAxis) {
  angle = constrain(angle, 0, 180);
  return reverseAxis ? 180 - angle : angle;
}

void writeGimbal(int yaw, int pitch) {
  yawAngle = constrain(yaw, YAW_MIN, YAW_MAX);
  pitchAngle = constrain(pitch, PITCH_MIN, PITCH_MAX);

  yawServo.write(mapServoAngle(yawAngle, YAW_REVERSE));
  pitchServo.write(mapServoAngle(pitchAngle, PITCH_REVERSE));

  Serial.print("yaw=");
  Serial.print(yawAngle);
  Serial.print(" pitch=");
  Serial.println(pitchAngle);
}

void printHelp() {
  Serial.println();
  Serial.println("2-DOF gimbal Nano test");
  Serial.println("Commands:");
  Serial.println("  c            center both servos");
  Serial.println("  w/s          pitch up/down");
  Serial.println("  a/d          yaw left/right");
  Serial.println("  x            run slow sweep test");
  Serial.println("  yaw,pitch    set absolute angles, for example: 90,90");
  Serial.println();
}

void sweepAxis(Servo &servo, int minAngle, int maxAngle, bool reverseAxis) {
  for (int angle = minAngle; angle <= maxAngle; angle += 2) {
    servo.write(mapServoAngle(angle, reverseAxis));
    delay(25);
  }
  for (int angle = maxAngle; angle >= minAngle; angle -= 2) {
    servo.write(mapServoAngle(angle, reverseAxis));
    delay(25);
  }
}

void runSweepTest() {
  Serial.println("Sweep yaw");
  sweepAxis(yawServo, YAW_MIN, YAW_MAX, YAW_REVERSE);
  writeGimbal(YAW_CENTER, PITCH_CENTER);
  delay(500);

  Serial.println("Sweep pitch");
  sweepAxis(pitchServo, PITCH_MIN, PITCH_MAX, PITCH_REVERSE);
  writeGimbal(YAW_CENTER, PITCH_CENTER);
}

void handleSerialLine(String line) {
  line.trim();
  if (line.length() == 0) {
    return;
  }

  if (line == "c") {
    writeGimbal(YAW_CENTER, PITCH_CENTER);
    return;
  }
  if (line == "x") {
    runSweepTest();
    return;
  }

  int commaIndex = line.indexOf(',');
  if (commaIndex > 0) {
    int yaw = line.substring(0, commaIndex).toInt();
    int pitch = line.substring(commaIndex + 1).toInt();
    writeGimbal(yaw, pitch);
    return;
  }

  char cmd = line.charAt(0);
  const int step = 5;

  if (cmd == 'a') {
    writeGimbal(yawAngle - step, pitchAngle);
  } else if (cmd == 'd') {
    writeGimbal(yawAngle + step, pitchAngle);
  } else if (cmd == 'w') {
    writeGimbal(yawAngle, pitchAngle + step);
  } else if (cmd == 's') {
    writeGimbal(yawAngle, pitchAngle - step);
  } else {
    printHelp();
  }
}

void setup() {
  Serial.begin(115200);

  yawServo.attach(YAW_PIN);
  pitchServo.attach(PITCH_PIN);
  delay(300);

  writeGimbal(YAW_CENTER, PITCH_CENTER);
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    handleSerialLine(line);
  }
}
