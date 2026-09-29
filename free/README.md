# Free-orientation IK

Interactive CLI for targets that specify only tool-tip XYZ. It searches wrist
pitch choices and selects the feasible solution closest to the current joint
pose. After each successful target, that solution becomes the starting pose
for the next target. Edit dimensions and the initial joint pose in
`kinematics.py`.

Run from the project root with `python -m free.kinematics`. Enter `x y z`
(space or comma separated) for each target; type `q` to quit. Search resolution
is set by `pitch_step` in the IK function (default 2 degrees).
