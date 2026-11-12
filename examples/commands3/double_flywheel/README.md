# Double flywheel (Commands v3)

The Commands v3 version of the [double flywheel example](../../commands2/double_flywheel/README.md): two YAMS
`FlyWheel`s, a lower and an upper wheel, in one mechanism (`DoubleFlyWheelMechanism`). Live tuning works the same
way, with one difference: the wheels' shared Live Tuning command is named after the mechanism, at
`/Tuning/DoubleFlyWheelMechanism/Live Tuning`. Pick `DoubleFlyWheelMechanism` as the Live Tuning command in the
YAMS Live Tuner.

The teleop bindings are in the `Double FlyWheel Teleop` opmode. While nothing else runs, both wheels coast (the
mechanism's idle command).

## Tests

- `DoubleFlyWheelLiveTuningTest` is the same Live Tuning test as the commands v2 example's.
- `DoubleFlyWheelShotTest` holds the right bumper in the `Double FlyWheel Teleop` opmode and checks that both wheels
  reach their shot speeds, within the ready tolerance, in a second, then coast down when it is let go.
