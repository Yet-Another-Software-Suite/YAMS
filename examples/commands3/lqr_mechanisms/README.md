# LQR mechanisms (Commands v3)

The Commands v3 version of the [LQR mechanisms example](../../commands2/lqr_mechanisms/README.md): an LQR arm and an
LQR flywheel, each its own mechanism (`LQRArmMechanism`, `LQRFlyWheelMechanism`). Live tuning works the same way.
Their Live Tuning commands are named after the mechanisms, at `/Tuning/LQRArmMechanism/Live Tuning` and
`/Tuning/LQRFlyWheelMechanism/Live Tuning`.

The bindings are in the `LQR Teleop` opmode. While nothing else runs, the flywheel coasts (its idle command).
