#include <Arduino.h>
#include <Servo.h>
#include <stdlib.h>
#include <string.h>

Servo shoulderServo;
Servo elbowServo;

const uint8_t shoulderServoPin = 9;
const uint8_t elbowServoPin = 6;
const size_t inputBufferSize = 16;

char inputBuffer[inputBufferSize];
size_t inputLength = 0;
bool inputOverflow = false;

bool parseAngle(char *text, int &angle) {
  if (*text == '\0') {
    return false;
  }

  char *end;
  long parsed = strtol(text, &end, 10);
  if (*end != '\0' || parsed < 0 || parsed > 180) {
    return false;
  }

  angle = static_cast<int>(parsed);
  return true;
}

void applyAngles(char *line) {
  char *separator = strchr(line, ',');
  if (separator == nullptr) {
    return;
  }

  *separator = '\0';
  int shoulderAngle;
  int elbowAngle;
  if (parseAngle(line, shoulderAngle) &&
      parseAngle(separator + 1, elbowAngle)) {
    shoulderServo.write(shoulderAngle);
    elbowServo.write(elbowAngle);
  }
}

void setup() {
  Serial.begin(115200);
  shoulderServo.attach(shoulderServoPin);
  elbowServo.attach(elbowServoPin);
  shoulderServo.write(90);
  elbowServo.write(90);
}

void loop() {
  while (Serial.available() > 0) {
    char incoming = static_cast<char>(Serial.read());

    if (incoming == '\r') {
      continue;
    }

    if (incoming == '\n') {
      if (!inputOverflow) {
        inputBuffer[inputLength] = '\0';
        applyAngles(inputBuffer);
      }
      inputLength = 0;
      inputOverflow = false;
    } else if (inputLength < inputBufferSize - 1) {
      inputBuffer[inputLength++] = incoming;
    } else {
      inputOverflow = true;
    }
  }
}
