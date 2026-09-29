# Free-orientation IK

This folder solves a tool-tip XYZ target without prescribing tool pitch. Since
many joint combinations can reach the same point, it samples wrist pitches
and selects the feasible solution with the least angle movement from a supplied
starting pose. Edit dimensions, `current`, and target XYZ in `kinematics.py`.
The pitch search resolution defaults to 2 degrees and can be changed with
`pitch_step`.

Run from the project root: `python -m free.kinematics`.
