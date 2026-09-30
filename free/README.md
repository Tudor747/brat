# Free-orientation IK

Interactive CLI for tool-tip XYZ targets. It samples possible wrist pitches and
selects the feasible solution closest to the current joint pose. Each successful
solution becomes the next starting pose.

Run from the project root with `python -m free.kinematics`. Enter `x y z`
(space or comma separated) repeatedly; type `q` to quit. The CLI is in
`kinematics.py`; the reusable solver is in `solver.py`.
