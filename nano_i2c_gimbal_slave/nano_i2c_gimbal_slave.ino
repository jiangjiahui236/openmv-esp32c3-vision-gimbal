#include <Servo.h>
#include <SoftwareSerial.h>
#include <Wire.h>

const byte I2C_ADDRESS = 0x12;

const byte YAW_PIN = 3;
const byte PITCH_PIN = 5;
const byte OPENMV_RX_PIN = 8;
const byte OPENMV_TX_PIN = 9;

const int YAW_CENTER = 90;
const int PITCH_CENTER = 90;

// Adjust these after testing the real mechanical limits.
const int YAW_MIN = 30;
const int YAW_MAX = 150;
const int PITCH_MIN = 45;
const int PITCH_MAX = 135;

const bool YAW_REVERSE = false;
const bool PITCH_REVERSE = false;

const unsigned long SERVO_UPDATE_INTERVAL_MS = 20;
const int SERVO_STEP_DEGREES = 2;
const unsigned long SERIAL_REPORT_INTERVAL_MS = 250;
const unsigned long OPENMV_TIMEOUT_MS = 1000;

Servo yawServo;
Servo pitchServo;
SoftwareSerial openmvSerial(OPENMV_RX_PIN, OPENMV_TX_PIN);

volatile int targetYaw = YAW_CENTER;
volatile int targetPitch = PITCH_CENTER;
int currentYaw = YAW_CENTER;
int currentPitch = PITCH_CENTER;

unsigned long lastServoUpdateMs = 0;
unsigned long lastSerialReportMs = 0;
unsigned long lastOpenMvCommandMs = 0;
char openmvLine[24];
byte openmvLineLength = 0;

int mapServoAngle(int angle, bool reverseAxis) {
  angle = constrain(angle, 0, 180);
  return reverseAxis ? 180 - angle : angle;
}

void writeGimbal(int yaw, int pitch) {
  currentYaw = constrain(yaw, YAW_MIN, YAW_MAX);
  currentPitch = constrain(pitch, PITCH_MIN, PITCH_MAX);

  yawServo.write(mapServoAngle(currentYaw, YAW_REVERSE));
  pitchServo.write(mapServoAngle(currentPitch, PITCH_REVERSE));
}

int moveToward(int current, int target, int maxStep) {
  if (current < target) {
    return min(current + maxStep, target);
  }
  if (current > target) {
    return max(current - maxStep, target);
  }
  return current;
}

void receiveEvent(int byteCount) {
  if (byteCount < 2) {
    while (Wire.available()) {
      Wire.read();
    }
    return;
  }

  int yaw = Wire.read();
  int pitch = Wire.read();

  while (Wire.available()) {
    Wire.read();
  }

  targetYaw = constrain(yaw, YAW_MIN, YAW_MAX);
  targetPitch = constrain(pitch, PITCH_MIN, PITCH_MAX);
}

void setTargetAngles(int yaw, int pitch) {
  noInterrupts();
  targetYaw = constrain(yaw, YAW_MIN, YAW_MAX);
  targetPitch = constrain(pitch, PITCH_MIN, PITCH_MAX);
  interrupts();
}

void handleOpenMvLine(char *line) {
  char *comma = strchr(line, ',');
  if (comma == NULL) {
    return;
  }

  *comma = '\0';
  int yaw = atoi(line);
  int pitch = atoi(comma + 1);
  setTargetAngles(yaw, pitch);
  lastOpenMvCommandMs = millis();
}

void readOpenMvSerial() {
  while (openmvSerial.available()) {
    char c = openmvSerial.read();

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      openmvLine[openmvLineLength] = '\0';
      if (openmvLineLength > 0) {
        handleOpenMvLine(openmvLine);
      }
      openmvLineLength = 0;
      continue;
    }

    if (openmvLineLength < sizeof(openmvLine) - 1) {
      openmvLine[openmvLineLength++] = c;
    } else {
      openmvLineLength = 0;
    }
  }
}

void requestEvent() {
  Wire.write((byte)currentYaw);
  Wire.write((byte)currentPitch);
}

void setup() {
  Serial.begin(115200);
  openmvSerial.begin(57600);

  yawServo.attach(YAW_PIN);
  pitchServo.attach(PITCH_PIN);
  delay(300);
  writeGimbal(YAW_CENTER, PITCH_CENTER);

  Wire.begin(I2C_ADDRESS);
  Wire.onReceive(receiveEvent);
  Wire.onRequest(requestEvent);

  Serial.println("Nano gimbal ready: I2C address 0x12, OpenMV serial on D8 RX");
}

void loop() {
  readOpenMvSerial();

  unsigned long now = millis();
  if (now - lastServoUpdateMs < SERVO_UPDATE_INTERVAL_MS) {
    return;
  }
  lastServoUpdateMs = now;

  noInterrupts();
  int yaw = targetYaw;
  int pitch = targetPitch;
  interrupts();

  if (yaw != currentYaw || pitch != currentPitch) {
    int nextYaw = moveToward(currentYaw, yaw, SERVO_STEP_DEGREES);
    int nextPitch = moveToward(currentPitch, pitch, SERVO_STEP_DEGREES);
    writeGimbal(nextYaw, nextPitch);
  }

  if (now - lastSerialReportMs >= SERIAL_REPORT_INTERVAL_MS) {
    lastSerialReportMs = now;
    Serial.print("yaw=");
    Serial.print(currentYaw);
    Serial.print(" pitch=");
    Serial.println(currentPitch);
    if (now - lastOpenMvCommandMs < OPENMV_TIMEOUT_MS) {
      Serial.println("OpenMV tracking active");
    }
  }
}
