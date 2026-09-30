/*
 * ESP stepper and gripper controller starter.
 *
 * This is an Arduino-style C++ sketch placeholder, not ready to drive hardware.
 * Choose the exact ESP board, motor drivers, pins, limit switches, and stepper
 * library before filling in the pin map and motion code.
 *
 * Suggested line protocol from the Raspberry Pi:
 *   MOVE,<base>,<shoulder>,<elbow>,<wrist_pitch>,<wrist_roll>\n
 * Values are joint angles in degrees. Add a separate GRIP,<value> command for
 * the gripper servo. Respond with OK or ERROR for every accepted/rejected line.
 */

#include <Arduino.h>

// Set these to match the specific ESP board and driver wiring.
const int BASE_STEP_PIN = -1;
const int BASE_DIR_PIN = -1;
const int SHOULDER_STEP_PIN = -1;
const int SHOULDER_DIR_PIN = -1;
const int ELBOW_STEP_PIN = -1;
const int ELBOW_DIR_PIN = -1;
const int WRIST_PITCH_STEP_PIN = -1;
const int WRIST_PITCH_DIR_PIN = -1;
const int WRIST_ROLL_STEP_PIN = -1;
const int WRIST_ROLL_DIR_PIN = -1;
const int GRIPPER_SERVO_PIN = -1;

void setup() {
  Serial.begin(115200);
  // TODO: configure pins, initialize the selected stepper/servo libraries,
  // initialize limit switches, and report a ready message.
}

void loop() {
  // TODO: read one complete newline-terminated command without blocking for
  // long periods; validate its fields, apply joint limits, then move safely.
  // Do not treat the placeholder pin values above as usable hardware settings.
}
