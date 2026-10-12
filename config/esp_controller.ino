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
// Two NEMA 17 motors drive a differential, not individual pitch/roll axes.
const int WRIST_A_STEP_PIN = -1;
const int WRIST_A_DIR_PIN = -1;
const int WRIST_B_STEP_PIN = -1;
const int WRIST_B_DIR_PIN = -1;
const int GRIPPER_SERVO_PIN = -1;

void setup() {
  Serial.begin(115200);
  // TODO: configure pins, initialize the selected stepper/servo libraries,
  // initialize limit switches, and report a ready message.
}

void loop() {
  // TODO: read one complete newline-terminated command without blocking for
  // long periods; validate its fields, apply joint limits, then move safely.
  // Mix joint targets into calibrated differential motor-shaft angles:
  // a = pitch_ratio * wrist_pitch + roll_ratio * wrist_roll
  // b = pitch_ratio * wrist_pitch - roll_ratio * wrist_roll
  // Then apply each motor's direction and steps-per-degree calibration.
  // Schedule A/B pulses together so the requested pitch/roll path is preserved.
  // Handle GRIP separately with the gripper servo.
  // Do not treat the placeholder pin values above as usable hardware settings.
}
