# Commissioning and IK calibration

## 1. Test the software and wiring

Run `python -m unittest discover -s tests -v`, then `python -m src.cli`.
Build/upload with ROBOARM_SIMULATION=true in config/hardware.h. Connect the serial CLI;
`status` must report SIM. Test enable, ref, joints, grip, stop, clear and rejection
of moves before reference. Firmware simulation exercises the real scheduler;
Python simulation completes moves instantly.

The flags are C++ preprocessor definitions: edit `#define ROBOARM_SIMULATION true`
and `#define ROBOARM_CALIBRATED false`, replacing only `true`/`false` as instructed.

For wiring checks set ROBOARM_SIMULATION=false but leave ROBOARM_CALIBRATED=false.
Upload; status must report HW. ENABLE/MOVE/REF/GRIP remain locked. Verify opening
each NC contact triggers FAULT,INTERLOCK, and CLEAR fails until all are closed.
Test the physical emergency-stop circuit independently. Validate the chosen
STEP/DIR driver interface and current before attaching the mechanism.

## 2. Measure geometry (config/raspberry_pi.py)

Set ARM = ArmDimensions(...) using mm measured between rotation axes, not casing
edges: base_height is table/origin to shoulder pivot; upper_arm is shoulder to
elbow pivot; forearm is elbow to wrist pitch pivot. wrist_length + tool_length
is wrist pitch pivot to tool tip along the tool axis. The IK only observes their
sum, so do not try to independently fit those two lengths from tip data.

Zero convention: base points along +X; shoulder, forearm and tool are horizontal
and collinear at pitch zeros. Positive pitch raises the next link; positive yaw
turns +X toward +Y. Elbow and wrist angles are RELATIVE, not absolute inclinations.
Using an inclinometer: shoulder = upper-arm inclination; elbow = forearm
inclination - shoulder; wrist = tool inclination - forearm inclination.

## 3. Establish motor scale and signs (config/hardware.h)

The example assumes 200 full steps/revolution, 16 microsteps, no reduction.
Replace these assumptions before enabling hardware:

    direct steps_per_degree = full_steps_per_rev * microsteps * reduction / 360

Reduction means motor revolutions per joint revolution. Wrist STEPS_PER_DEGREE
uses motor shaft angle; differential gearing goes in WRIST_PITCH_RATIO and
WRIST_ROLL_RATIO (motor shaft degrees per joint degree):

    A = pitch_ratio * wrist_pitch + roll_ratio * wrist_roll
    B = pitch_ratio * wrist_pitch - roll_ratio * wrist_roll
    steps[i] = round(motor_degrees[i] * steps_per_degree[i] * direction[i])

Do not count wrist gearing twice. The symmetric differential model must match
your mechanism. If A/B need unequal coupling ratios, adapt the mixing model
before using IK. NEMA17/23 alone does not specify any calibration value.

Support the arm, remove loads, and begin with small conservative travel limits
around a measured test pose in BOTH hardware.h and raspberry_pi.py. Set low
MAX_STEP_RATE and STEP_ACCELERATION. After checking wiring and estimated scales,
set ROBOARM_CALIBRATED=true (ROBOARM_SIMULATION=false), rebuild/upload. This flag is
your explicit acknowledgement that the physical setup is ready for small tests.

At a known supported pose, `enable`, remeasure the pose after engagement, then
`ref b s e p r`. REF only assigns coordinates; it is not homing and must not be
used to pretend an unknown pose is zero. Never forcibly turn an energized motor.

Command a small change (initially 1 degree) to one direct joint with `joints`,
keeping every other angle equal to its reference. Check physical sign first.
If reversed, change DIRECTION for that motor, rebuild and reference again.
For measurable larger motion inside the safe range:

    corrected_steps_per_degree = old_steps_per_degree * commanded_delta / measured_delta

Use repeated tests in both directions. Different results indicate backlash,
missed steps, compliance or a measurement issue; do not hide these with one scale.

## 4. Calibrate the differential wrist

With the mechanism supported, first verify A/B motor directions in the calibrated
convention. Command small pure pitch (roll unchanged): the tool should pitch
without rolling. Command small pure roll (pitch unchanged): it should roll
without pitching. Wrong cross-motion usually indicates one sign or channel is
wrong. Do not compensate with tool lengths.

    new_pitch_ratio = old_pitch_ratio * commanded_pitch_delta / measured_pitch_delta
    new_roll_ratio = old_roll_ratio * commanded_roll_delta / measured_roll_delta

Remeasure/ref after every firmware/calibration change. Verify combined pitch and
roll; both motors must remain synchronized. Adjust each motor's scale if the
motors have different microstep settings. Keep cables within permitted roll.

## 5. Limits and gripper

Measure comfortable joint ranges short of hard stops and cable tension. Copy
those limits into JOINT_MIN/MAX (firmware) and JOINT_LIMITS (host). Interpolated
joint positions stay inside these rectangular limits, but collisions can still
occur between valid endpoints. Choose waypoints accordingly.

Set servo pulse endpoints conservatively with the gripper unloaded. `grip 0`
and `grip 1` mean endpoint pulses, not universally open/closed. Determine which
is open on your linkage and narrow the pulse range before increasing travel.
Stop removes servo PWM; it cannot guarantee the gripper holds an object.

## 6. Validate IK against measurements

Start with joint commands. At several safe poses, record the five actual angles,
`fk` predicted XYZ/pitch and measured XYZ/pitch. Measure XYZ in the same base
frame. Then run fixed-pitch `xyz x y z pitch roll` for nearby positions. Only
try free-pitch `xyz x y z` once individual axes and fixed IK agree.

| Pose | Reference/measured angles | FK XYZ | Measured XYZ | XYZ error |
|---|---|---|---|---|
| A | | | | |
| B | | | | |
| C | | | | |

A constant offset suggests the origin/base height; error growing with rotation
suggests angle zero/scale; error changing with extension suggests link lengths;
direction-dependent error suggests backlash. Change one parameter at a time.
Recheck at independent poses, including both elbow configurations when physically
safe. Define your acceptable positional error from the actual task and measure
repeatability under load before raising speed.

There is no automatic numerical calibration or automatic homing in this package.
Calibration consists of measuring dimensions, motor scales, signs, reference
poses and limits. Each power cycle/reset/stop requires remeasurement and REF.
The example dimensions and limits are starting values, not hardware certification.
