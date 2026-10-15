#pragma once
// Classic ESP32-WROOM-32 DevKit, not WROVER/PSRAM, C3 or S3.
// Edit these two defaults when commissioning. Build flags may override them.
#ifndef ROBOARM_SIMULATION
#define ROBOARM_SIMULATION true
#endif
#ifndef ROBOARM_CALIBRATED
#define ROBOARM_CALIBRATED false
#endif
constexpr bool SIMULATION = ROBOARM_SIMULATION;
constexpr bool CALIBRATION_CONFIRMED = ROBOARM_CALIBRATED;
constexpr int STEP_PINS[5] = {13, 16, 18, 21, 25};
constexpr int DIR_PINS[5] = {14, 17, 19, 22, 26};
constexpr int ENABLE_PIN = 27; // shared active LOW; external 10k pullup
constexpr int SERVO_PIN = 23;
// NC contacts to GND; external 10k pullups to 3.3V on ALL inputs.
constexpr int LIMIT_PINS[5] = {32, 33, 34, 35, 36};
constexpr int ESTOP_PIN = 39;
// Direct axes include gearing; wrist values are motor-shaft steps/degree.
constexpr double STEPS_PER_DEGREE[5] = {8.888888889, 8.888888889, 8.888888889, 8.888888889, 8.888888889};
constexpr int DIRECTION[5] = {1, 1, 1, 1, 1};
constexpr double WRIST_PITCH_RATIO = 1.0;
constexpr double WRIST_ROLL_RATIO = 1.0;
constexpr double JOINT_MIN[5] = {-90, -30, -120, -90, -90};
constexpr double JOINT_MAX[5] = {90, 120, 120, 90, 90};
constexpr double MAX_STEP_RATE = 500.0;
constexpr double STEP_ACCELERATION = 200.0;
constexpr unsigned STEP_PULSE_US = 5;
constexpr unsigned LINK_TIMEOUT_MS = 2000;
constexpr int SERVO_MIN_US = 1000;
constexpr int SERVO_MAX_US = 2000;

static_assert(MAX_STEP_RATE > 0 && MAX_STEP_RATE <= 5000, "Step rate must be 0..5000 events/s");
static_assert(STEP_ACCELERATION > 0, "Acceleration must be positive");
static_assert(WRIST_PITCH_RATIO > 0 && WRIST_ROLL_RATIO > 0, "Ratios must be positive");
static_assert(SERVO_MIN_US >= 500 && SERVO_MAX_US <= 2500 && SERVO_MIN_US < SERVO_MAX_US,
              "Servo pulse endpoints invalid");
static_assert(STEP_PULSE_US >= 1 && STEP_PULSE_US <= 50, "Pulse width must be 1..50us");
constexpr bool validAxes() {
  for (int i = 0; i < 5; ++i)
    if (!(STEPS_PER_DEGREE[i] > 0) || !(JOINT_MIN[i] < JOINT_MAX[i]) ||
        (DIRECTION[i] != 1 && DIRECTION[i] != -1)) return false;
  return true;
}
static_assert(validAxes(), "Invalid motor scale, joint limits or direction");
