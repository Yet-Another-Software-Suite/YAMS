# LQR mechanisms

An arm and a flywheel run by LQRs instead of PID controllers, each in its own subsystem:

| Subsystem | Motor | Mechanism |
| --- | --- | --- |
| `LQRArmSubsystem` | NEO on a 60:1 reduction (CAN 20) | 3 kg, 0.5 m arm, from −20° (stowed) to 100° |
| `LQRFlyWheelSubsystem` | NEO, direct drive (CAN 21) | 4 in, 1 lb wheel |

It is also the test program for live tuning LQRs with the [YAMS Live Tuner](https://tuning.yassrobotics.com/).

## Live tuning an LQR

A motor controller configured with an LQR publishes the LQR's values in place of kP, kI and kD, with HIGH telemetry:

| Field, under `closedloop/lqr/` | What it is | Arm | Flywheel |
| --- | --- | --- | --- |
| `qelms/position` | Position error tolerance | degrees | |
| `qelms/velocity` | Velocity error tolerance | RPM | RPM |
| `relms` | Control effort (voltage) tolerance | volts | volts |
| `model/position` | Model position standard deviation | degrees | |
| `model/velocity` | Model velocity standard deviation | RPM | RPM |
| `encoder/position` | Encoder standard deviation | degrees | |
| `encoder/velocity` | Encoder standard deviation | | RPM |
| `maxVoltage` | Maximum output voltage | volts | volts |

- **Units:** the same as the tunable setpoints. An elevator's are in meters and meters per second.
- **Retuning:** while the Live Tuning command runs, a change rebuilds the LQR. The new LQR starts from the old one's state estimate, so its output doesn't jump.
- **Bad values:** a value the LQR can't use, such as a tolerance of 0, leaves it as it was, with a warning on the Driver Station.
- **Restarting the closed loop:** an LQR runs on the SystemCore, and a duty cycle or voltage command stops it, like the flywheel's coasting default command does. Live Tuning starts it again.

In the tuner, the Settings card shows these as an **LQR** group. The tuner shows an arm's velocities in degrees per second and converts them to RPM.

## Controls

- Operator controller, A: raise the arm level.
- Operator controller, B: stow the arm.
- Operator controller, right bumper: spin the flywheel up to 3000 RPM.

## Tests

`LQRLiveTuningTest` drives both Live Tuning commands over NetworkTables the way the tuner does. It checks that:

- an LQR replaces kP, kI and kD with the fields its type uses;
- the fields start at the configured values;
- the flywheel reaches its speed;
- changing a field retunes each LQR while it holds its setpoint;
- a value the LQR can't use is not applied.
