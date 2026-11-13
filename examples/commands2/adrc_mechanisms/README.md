# ADRC mechanisms

An arm, an elevator and a shooter run by active disturbance rejection controllers (ADRCs) instead of PID controllers,
each in its own subsystem. **The ADRC was written by FRC Team 8331**, who contributed it to YAMS. The
[ADRC guide](../../../ADRCGuide.md) explains how it works and how to choose its settings.

| Subsystem | Motor | Mechanism | ADRC |
| --- | --- | --- | --- |
| `ADRCArmSubsystem` | NEO on a 60:1 reduction (CAN 20) | 3 kg, 0.5 m arm, from -20° (stowed) to 100° | `ARM` on position, 0.3 s settling time; at 60:1 it is first order with its speed per volt as `b0` |
| `ADRCElevatorSubsystem` | NEO on a 12:1 reduction (CAN 21) | 16 lb carriage on a 22 tooth, 1/4 in pitch sprocket, 0 to 1.5 m | `ELEVATOR`: first order on position, 0.3 s settling time, `b0` its speed per volt |
| `ADRCShooterSubsystem` | NEO, direct drive (CAN 22) | 4 in, 1 lb wheel | `FLYWHEEL` (first order) on velocity, 0.2 s settling time, `b0` derived from the inertia |

## Setting up an ADRC

`withADRCController(...)` on the motor config takes the place of `withClosedLoopController(kP, kI, kD)` and
`withFeedforward(...)`:

```java
new SmartMotorControllerConfig(this)
    .withControlMode(ControlMode.CLOSED_LOOP)
    .withADRCController(new ADRCConfig()
        .withSystemOrder(SystemOrder.ARM)
        .withSettlingTime(Seconds.of(0.3)))
    .withClosedLoopControllerMaximumVoltage(Volts.of(12))
    .withGearing(new MechanismGearing(60))
    .withMomentOfInertia(Meters.of(0.5), Kilograms.of(3)); // b0 is derived from this
```

The mechanisms themselves need no changes: `Arm.setAngle`, `Elevator.setHeight` and `FlyWheel.run` set their
setpoints as they do with a PID, and the motor controller runs the ADRC on the SystemCore.

- **No kG, kS or kV.** The ADRC's observer estimates gravity, friction, back-EMF and anything that bumps the mechanism
  as one disturbance and cancels it.
- **The elevator is `SystemOrder.ELEVATOR`.** An elevator's speed is set by its back-EMF, so its position is treated
  as a first order plant with its speed per volt as `b0`, which is derived from the motor, gearing and sprocket.
- **The elevator's closed loop is linear.** `withLinearClosedLoopController(true)` runs the closed loop, and the ADRC,
  in meters, so `b0` is in meters per second per volt. `SystemOrder.ELEVATOR` needs it: without it, the motor
  controller refuses the config.
- **The shooter's settling time is short,** so the observer keeps up with the back-EMF it estimates. A short settling
  time also makes it recover quickly after each shot. Shorter than 0.2 s, it overshoots: about 45 RPM at 0.15 s.
- **Each settling time was picked by measuring.** The [ADRC guide's tuning section](../../../ADRCGuide.md#tuning-an-adrc)
  shows how.

## Live tuning an ADRC

With HIGH telemetry, a motor controller with an ADRC publishes its settings in place of kP, kI and kD, under
`closedloop/adrc/`, and the [YAMS Live Tuner](https://tuning.yassrobotics.com/) can change them:

| Field | What it is |
| --- | --- |
| `b0` | How much one volt moves the plant's highest derivative |
| `settlingTime` | How quickly the mechanism should reach its setpoint, seconds |
| `bandwidthScale` | Multiplies the controller and observer bandwidths |

The output limit and ramp are the motor config's, as for every closed loop: `withClosedLoopControllerMaximumVoltage`
and `withClosedLoopRampRate`, which ramps an ADRC's output too.

A change is stored in the motor controller's `ADRCConfig`, which the ADRC reads its settings and gains from, and the
config works the gains out again, so the ADRC runs the change on its next loop. The estimated disturbance carries over,
so the output doesn't jump. The config refuses a value the ADRC
can't use, such as a settling time of 0: every value stays as it was, and an alert is raised.

## Controls

- Operator controller, A: raise the arm level.
- Operator controller, B: stow the arm.
- Operator controller, Y: raise the elevator to 1 m.
- Operator controller, X: lower the elevator.
- Operator controller, right bumper: spin the shooter up to 3000 RPM.

## Tests

`ADRCMechanismsTest` runs the robot in simulation. It checks that:

- an ADRC replaces kP, kI and kD with its own fields, starting at the configured values;
- the arm, the elevator and the shooter each reach their setpoints;
- changing the arm's settling time over NetworkTables, the way the tuner does, retunes its ADRC while it holds its
  angle.
