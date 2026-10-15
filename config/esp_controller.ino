// Five-axis STEP/DIR controller. See README.md and CALIBRATION.md.
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "hardware.h"

long positionSteps[5] = {}, deltaSteps[5] = {}, errorSteps[5] = {};
int stepSign[5] = {};
double joints[5] = {}, destination[5] = {};
long totalTicks = 0, completedTicks = 0;
bool moving = false, referenced = false, enabled = false, faulted = false;
uint32_t nextTick = 0, lastContact = 0;
char inputLine[192];
size_t inputLength = 0;
bool overflowed = false;

void replyError(const char *message) { Serial.print("ERROR,"); Serial.println(message); }
bool interlock() {
  if (SIMULATION) return false;
  if (digitalRead(ESTOP_PIN) == HIGH) return true;
  for (int pin : LIMIT_PINS) if (digitalRead(pin) == HIGH) return true;
  return false;
}
void stopMotion(const char *reason) {
  moving = false; referenced = false; faulted = true;
  // Preserve holding torque; DISABLE is an explicit separate action.
  if (!SIMULATION) ledcWrite(0, 0);
  Serial.print("FAULT,"); Serial.println(reason);
}
bool parseValues(char *args, double *values, int count) {
  if (!args) return false;
  for (int i = 0; i < count; ++i) {
    char *end;
    values[i] = strtod(args, &end);
    if (end == args || !isfinite(values[i])) return false;
    if (i == count - 1) return *end == '\0';
    if (*end != ',') return false;
    args = end + 1;
  }
  return false;
}
bool convert(const double *values, long *steps) {
  for (int i = 0; i < 5; ++i)
    if (values[i] < JOINT_MIN[i] || values[i] > JOINT_MAX[i]) return false;
  double motors[5] = {values[0], values[1], values[2],
    WRIST_PITCH_RATIO * values[3] + WRIST_ROLL_RATIO * values[4],
    WRIST_PITCH_RATIO * values[3] - WRIST_ROLL_RATIO * values[4]};
  for (int i = 0; i < 5; ++i) {
    double result = motors[i] * STEPS_PER_DEGREE[i] * DIRECTION[i];
    if (!isfinite(result) || fabs(result) > 10000000) return false;
    steps[i] = lround(result);
  }
  return true;
}
void command(char *line) {
  char *args = strchr(line, ',');
  if (args) *args++ = '\0';
  if (!strcmp(line, "PING") && !args) {
    lastContact = millis(); Serial.println("OK,PING"); return;
  }
  if (!strcmp(line, "STATUS") && !args) {
    Serial.printf("STATUS,%s,%s,%d,%d,%d", SIMULATION ? "SIM" : "HW",
                  moving ? "MOVING" : "IDLE", referenced, enabled, faulted);
    for (double angle : joints) { Serial.print(','); Serial.print(angle, 5); }
    Serial.println(); return;
  }
  if (!strcmp(line, "STOP") && !args) {
    stopMotion("STOP"); Serial.println("OK,STOP"); return;
  }
  if (!strcmp(line, "DISABLE") && !args) {
    stopMotion("DISABLED"); enabled = false;
    if (!SIMULATION) digitalWrite(ENABLE_PIN, HIGH);
    Serial.println("OK,DISABLE"); return;
  }
  if (moving) { replyError("BUSY"); return; }
  if (!strcmp(line, "CLEAR") && !args) {
    if (interlock()) { replyError("INTERLOCK"); return; }
    faulted = false; Serial.println("OK,CLEAR"); return;
  }
  if (faulted || interlock()) { replyError("INTERLOCK_OR_FAULT"); return; }
  if (!SIMULATION && !CALIBRATION_CONFIRMED) { replyError("UNCALIBRATED"); return; }
  if (!strcmp(line, "ENABLE") && !args) {
    enabled = true;
    if (!SIMULATION) digitalWrite(ENABLE_PIN, LOW);
    Serial.println("OK,ENABLE"); return;
  }
  if (!strcmp(line, "REF") || !strcmp(line, "MOVE")) {
    double values[5]; long steps[5];
    if (!parseValues(args, values, 5)) { replyError("FORMAT"); return; }
    if (!convert(values, steps)) { replyError("LIMIT"); return; }
    if (!enabled) { replyError("DISABLED"); return; }
    if (!strcmp(line, "REF")) {
      for (int i = 0; i < 5; ++i) { positionSteps[i] = steps[i]; joints[i] = values[i]; }
      referenced = true; Serial.println("OK,REF"); return;
    }
    if (!referenced) { replyError("REFERENCE_REQUIRED"); return; }
    totalTicks = completedTicks = 0;
    for (int i = 0; i < 5; ++i) {
      destination[i] = values[i]; deltaSteps[i] = labs(steps[i] - positionSteps[i]);
      stepSign[i] = steps[i] >= positionSteps[i] ? 1 : -1; errorSteps[i] = 0;
      if (deltaSteps[i] > totalTicks) totalTicks = deltaSteps[i];
      if (!SIMULATION) digitalWrite(DIR_PINS[i], stepSign[i] > 0 ? HIGH : LOW);
    }
    lastContact = millis(); nextTick = micros() + 20;
    Serial.println("OK,MOVE"); moving = totalTicks > 0;
    if (!moving) { for (int i = 0; i < 5; ++i) joints[i] = destination[i]; Serial.println("DONE,MOVE"); }
    return;
  }
  if (!strcmp(line, "GRIP")) {
    double value[1];
    if (!parseValues(args, value, 1) || value[0] < 0 || value[0] > 1) { replyError("FORMAT"); return; }
    if (!enabled) { replyError("DISABLED"); return; }
    int pulse = lround(SERVO_MIN_US + value[0] * (SERVO_MAX_US - SERVO_MIN_US));
    if (!SIMULATION) ledcWrite(0, (uint32_t)((double)pulse * 65535 / 20000));
    Serial.println("OK,GRIP"); return;
  }
  replyError("UNKNOWN_COMMAND");
}
void tickMotion() {
  if (!moving) return;
  if ((uint32_t)(millis() - lastContact) > LINK_TIMEOUT_MS) { stopMotion("LINK_TIMEOUT"); return; }
  if ((int32_t)(micros() - nextTick) < 0) return;
  bool pulse[5] = {};
  for (int i = 0; i < 5; ++i) {
    errorSteps[i] += deltaSteps[i];
    if (errorSteps[i] >= totalTicks) {
      errorSteps[i] -= totalTicks; pulse[i] = true; positionSteps[i] += stepSign[i];
      if (!SIMULATION) digitalWrite(STEP_PINS[i], HIGH);
    }
  }
  if (!SIMULATION) delayMicroseconds(STEP_PULSE_US);
  for (int i = 0; i < 5; ++i) if (pulse[i] && !SIMULATION) digitalWrite(STEP_PINS[i], LOW);
  ++completedTicks;
  if (completedTicks == totalTicks) {
    moving = false;
    for (int i = 0; i < 5; ++i) joints[i] = destination[i];
    Serial.println("DONE,MOVE"); return;
  }
  long rampTicks = min(completedTicks, totalTicks - completedTicks);
  double rate = fmin(MAX_STEP_RATE, sqrt(2.0 * STEP_ACCELERATION * rampTicks));
  nextTick = micros() + (uint32_t)(1000000.0 / rate);
}
void setup() {
  Serial.begin(115200);
  if (!SIMULATION) {
    digitalWrite(ENABLE_PIN, HIGH); pinMode(ENABLE_PIN, OUTPUT);
    for (int i = 0; i < 5; ++i) {
      pinMode(STEP_PINS[i], OUTPUT); digitalWrite(STEP_PINS[i], LOW);
      pinMode(DIR_PINS[i], OUTPUT); pinMode(LIMIT_PINS[i], INPUT);
    }
    pinMode(ESTOP_PIN, INPUT);
    // Pinned Arduino-ESP32 2.x uses channel-based LEDC API.
    ledcSetup(0, 50, 16); ledcAttachPin(SERVO_PIN, 0); ledcWrite(0, 0);
  }
  Serial.println(SIMULATION ? "READY,1,SIM" : "READY,1,HW");
}
void loop() {
  if (!faulted && interlock()) stopMotion("INTERLOCK");
  for (int budget = 0; budget < 32 && Serial.available(); ++budget) {
    char c = Serial.read();
    if (c == '\n') {
      if (overflowed) replyError("LINE_TOO_LONG");
      else { inputLine[inputLength] = '\0'; command(inputLine); }
      inputLength = 0; overflowed = false;
    } else if (c != '\r') {
      if (c == '\0' || inputLength >= sizeof(inputLine) - 1) overflowed = true;
      else if (!overflowed) inputLine[inputLength++] = c;
    }
  }
  tickMotion();
}
