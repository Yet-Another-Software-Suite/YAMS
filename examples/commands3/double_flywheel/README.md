# Double flywheel (Commands v3)

The Commands v3 version of the [double flywheel example](../../commands2/double_flywheel/README.md): two YAMS
`FlyWheel`s, a lower and an upper wheel, in one mechanism (`DoubleFlyWheelMechanism`). Live tuning works the same
way, with one difference: the wheels' shared Live Tuning command is named after the mechanism, at
`/Tuning/DoubleFlyWheelMechanism/Live Tuning`. Pick `DoubleFlyWheelMechanism` as the Live Tuning command in the
YAMS Live Tuner.

The teleop bindings are in the `Double FlyWheel Teleop` opmode. While nothing else runs, both wheels coast (the
mechanism's idle command).
