# Double flywheel

A robot with one double flywheel: two YAMS `FlyWheel`s, a lower and an upper wheel, in one subsystem
(`DoubleFlyWheelSubsystem`). Running the wheels at different speeds puts spin on the game piece. Each wheel has its
own NEO on a SPARK MAX (CAN 10 and 11), with the upper motor inverted to face the lower wheel.

It is also the test program for tuning a double flywheel with the [YAMS Live Tuner](https://tuning.yassrobotics.com/).

## How live tuning works with two wheels

Both wheels publish HIGH telemetry, so each one registers its live tuning on the subsystem they share. They get one
**Live Tuning** command between them, at `/Tuning/DoubleFlyWheelSubsystem/Live Tuning`. While it runs, each wheel
applies the values under its own table:

| Wheel | Telemetry | Tunables |
| --- | --- | --- |
| Lower | `/Mechanisms/LowerFlyWheel/LowerMotor` | `/Tuning/LowerFlyWheel/LowerMotor` |
| Upper | `/Mechanisms/UpperFlyWheel/UpperMotor` | `/Tuning/UpperFlyWheel/UpperMotor` |

So each wheel keeps its own gains, setpoint and voltage, and starting or stopping the one command starts or stops
tuning both.

## Tuning it with the YAMS Live Tuner

1. Run the robot in simulation, or deploy it, and enable it in Teleop.
2. In the tuner, connect, then on the **Mechanism** step:
   - **Telemetry name**: `LowerFlyWheel/LowerMotor`.
   - **Type**: **Flywheel**. While the wheels coast, nothing they publish tells a flywheel from a pivot, so the
     tuner guesses the type that has its range checked.
   - **Second flywheel**: `UpperFlyWheel/UpperMotor`.
   - **Live Tuning command**: `DoubleFlyWheelSubsystem`. The tuner picks it when it is the only command no
     mechanism is named after.
3. The inversion check drives one wheel at a time. It passes once both wheels have.
4. On **Live tune**:
   - **Setpoints**: set each wheel's speed and send both at once, or move one wheel's slider.
   - **Voltage**, **Settings** and the telemetry readouts work on the wheel chosen with the **Wheel** picker.
   - The chart shows both wheels, the upper one dashed.
   - **Copy these gains** on the Settings card copies one wheel's gains to the other.

The gains start low on purpose (kV 0.1 V per rotation per second, where about 0.127 holds a NEO at speed), so both
wheels settle short of their setpoints until they are tuned. Copy the tuned values back into
`DoubleFlyWheelSubsystem` when you are done.

## Controls

- Operator controller, right bumper: spin up for a shot (3000 RPM lower, 2000 RPM upper).
- Operator controller, Start: start or stop the Live Tuning command from code.
- Default: both wheels coast.

## Tests

`DoubleFlyWheelLiveTuningTest` drives the Live Tuning command over NetworkTables the way the tuner does. It checks
that:

- the subsystem publishes one Live Tuning command;
- each wheel follows its own setpoint;
- each wheel has its own gains;
- a voltage drives only its wheel;
- stopping Live Tuning turns off every wheel's voltage.
