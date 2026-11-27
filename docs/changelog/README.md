---
description: New updates and improvements
icon: clock-rotate-left
layout:
  width: wide
  title:
    visible: true
  description:
    visible: true
  tableOfContents:
    visible: false
  outline:
    visible: true
  pagination:
    visible: false
  metadata:
    visible: true
  tags:
    visible: true
  actions:
    visible: true
  anchors:
    visible: true
---

# Changelog

## 2026.11.25

Pre-release, November 25, 2026 ([release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.11.25)). Download the example robot projects from the release. See the [migration guide](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/MIGRATION.md#renamed-and-removed-apis).

### Added

* `SwerveDrive.getRotation()` (C++ `GetRotation()`): the robot's rotation from odometry, the heading of `getPose()`.
* `SwerveDrive.zeroOdometryHeading()` (C++ `ZeroOdometryHeading()`) turns the pose to face 0, so field relative "forward" is the way the robot faces. `zeroGyro()` does the same.
* `SwerveDriveConfig.withRotation(Supplier<Rotation3d>)` (C++ `WithRotation`) sets the gyro. `withGyro` does the same, so existing code keeps working.
* Swerve drive telemetry: `rotation`, the robot's rotation from odometry, used for the module states, as a `Rotation2d`, with `SwerveDriveTelemetryConfig.withRotation()` (C++ `WithRotation()`). It's on from `LOW`.

### Changed

* **Behavior change:** field relative driving (`setFieldRelativeChassisSpeeds`), `getFieldRelativeSpeed()`, driving to a pose and `SwerveInputStream`'s heading, aim and locked-heading control use the robot's rotation from odometry, `getRotation()`, instead of the gyro, in Java and C++. Vision measurements that correct the pose's heading now also correct which way "forward" is. Without vision, it is the gyro's heading, so nothing changes. If vision's heading is noisy, give it a larger heading standard deviation. See [Gyro integration](https://yams.yassrobotics.com/documentation/understanding/swerve-drive#gyro-integration-and-field-vs.-robot-relative-speeds).
* **Behavior change:** `resetOdometry(Pose2d)` (C++ `ResetOdometry`) resets only the pose, and no longer changes the gyro offset or the simulated gyro. `getGyroRotation3d()` and the `gyro` telemetry keep the gyro's reading after a reset; `getRotation()`, which field relative driving uses, reads the reset pose's heading. `zeroGyro()` resets the pose's heading the same way.
* **Breaking (telemetry):** a swerve drive's `gyro` is a `Rotation2d` instead of a `double` in degrees, in Java and C++. Dashboards and AdvantageScope layouts that read it as a number need to read the `Rotation2d`. `DoubleTelemetryField.Gyro` is replaced by `StructTelemetryField.Gyro`. See [Telemetry verbosity](https://yams.yassrobotics.com/documentation/understanding/swerve-drive#telemetry-verbosity).

### Removed

* **Breaking:** `SwerveDriveConfig.withGyroOffset(Angle)` and `getGyroOffset()` (C++ `WithGyroOffset`, `GetGyroOffset`). The pose estimator keeps the offset between the gyro and the pose, so set where the robot faces with the starting pose, `resetOdometry(Pose2d)` or `zeroOdometryHeading()`.

## 2026.11.24

Released November 24, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.11.24).

### Added

* Rivet says hi on the Driver Station display the first time a motor controller config or swerve drive is built, with the YAMS version. The docked Display panel shows a short welcome, and popping it out shows Rivet. Lines your own code writes to the display replace it. `Rivet.getVersion()` (C++ `Rivet::GetVersion()`) gives the YAMS version, or `dev` when built from source.
* Swerve drive telemetry: `.type`, which reads `SwerveDrive` so a dashboard can recognize the drive's table, and `maxModuleVelocity`, the modules' maximum speed in m/s, with `withMaximumModuleVelocity()` (C++ `WithMaximumModuleVelocity()`). See [Telemetry verbosity](https://yams.yassrobotics.com/documentation/understanding/swerve-drive#telemetry-verbosity).
* Simulation: each motor controller raises an alert while it is simulated with the default moment of inertia, 0.02 kg·m².
* **C++:** YAMS and YAGSL report their usage to the HAL, as Java does.

### Changed

* **Breaking:** setting a motor controller config's moment of inertia a second time throws a `SmartMotorControllerConfigurationException`, in Java and C++. A simulation reads it once, when it is built, so a second value was never used. `withMomentOfInertia`, `withWheelMomentOfInertia` and the arm or flywheel `withExponentialProfile(Voltage, DCMotor, MomentOfInertia)` all set it, so use only one. A copy made with `clone()` can set its own once. See [`withMomentOfInertia`](https://yams.yassrobotics.com/documentation/reference/editor/config/simulation#withmomentofinertia).
* **Behavior change (Thrifty Nova):** a feedforward without a trapezoidal profile runs on the SystemCore, with its PID, and an alert says why. ThriftyBot confirmed the Nova only applies its motion profile slot's kS, kV, kA and kG in a motion profile, so in v2026.11.14 such a feedforward was sent to the Nova and never applied. A feedforward whose gains are all 0 counts, as live tuning can change them, and so does one in `SLOT_1`. Add `withTrapezoidalProfile` to run the loop and feedforward on the Nova. This moves a Nova swerve drive motor or flywheel with kS and kV, and an arm with kG but no profile, to the SystemCore, so check their PID gains. A simulated Nova applies its feedforward only in a motion profile too. See [Where the loop runs](https://yams.yassrobotics.com/documentation/reference/editor/config/closed-loop-control#where-the-loop-runs).
* **Behavior change:** swerve drive telemetry at `LOW` publishes everything a dashboard's swerve drive widget needs: `states/current` and `chassis/current`, which were at `MID`, and `states/desired` and `chassis/desired`, which were at `HIGH`, with `maxModuleVelocity`. `chassis/field` stays at `MID`.

### Fixed

* DataLog entries under a `withDataLogName(...)` name, such as YAGSL's `Swerve/`, were stamped in seconds where WPILib 2027 expects nanoseconds, so every value in a run landed at nearly the same instant and AdvantageScope couldn't play the run back. They are stamped on the DataLog's own clock, like its NetworkTables entries. C++ was already right.

## 2026.11.14

Released November 14, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.11.14).

### Added

* **A301 followers:** an A301 can be followed by other `A301Wrapper`s, with `withFollowers(Pair.of(follower, inverted))`. The A301 has no follower mode, so the main A301 sends each follower its voltage and duty cycle. Its `getDCMotor()` is `YMotors.getA301(n)` with every A301, its `simIterate()` iterates the followers, and a follower reads its simulation. See [Several A301s on one mechanism](https://yams.yassrobotics.com/api-reference/java-reference/motor-controllers/a301-wrapper#several-a301s-on-one-mechanism).
* **`getMechanismDCMotor()`:** the `DCMotor` of every motor on a mechanism, a motor controller's own and its loosely coupled followers'. The mechanism classes simulate it.
* **`getMechanismDutyCycle(double)`:** the duty cycle a mechanism's simulation applies, each motor controller's own weighted by its share of the torque. The simulation suppliers use it.
* **ADRC, written by team 8331:** `withADRCController(ADRCConfig)` (C++ `WithADRCController`) closes the loop with an Active Disturbance Rejection Controller instead of a PID and feedforward, on every motor controller, on the SystemCore. An extended state observer estimates gravity, friction, back-EMF and outside loads as one disturbance and cancels it, so there is no kS, kV, kG or kA to tune. `ADRCConfig` takes the system order (`FLYWHEEL`, or `POSITION`, with `ARM`, `PIVOT` and `ELEVATOR` as aliases, which picks a first or second order loop from the motor's back-EMF time constant) and a settling time, and works out `b0` from the motor, gearing and moment of inertia unless it is set. It can follow a motion profile, or shape the setpoint with a tracking differentiator. Live tuning publishes `b0`, the settling time and the bandwidth scale under `closedloop/adrc/`, and a value the controller can't use is refused with an alert. See [How do I use ADRC instead of PID?](https://yams.yassrobotics.com/documentation/how-to-tuning/how-do-i-use-adrc).
* Telemetry: `rotor/gearRatio`, the rotor rotations per mechanism rotation, at every level, and `measurement/circumference`, a linear mechanism's meters per mechanism rotation, from `LOW` (Java and C++). A log can turn a rotor reading into the mechanism's without working out the gear ratio by hand.
* Examples, in Commands v2 and v3: `adrc_mechanisms`, an arm, an elevator and a shooter each closed with an ADRC. The C++ example has `ADRCArmSubsystem`, `ADRCElevatorSubsystem` and `ADRCShooterSubsystem`, commented out in `RobotContainer`.

### Changed

* **Behavior change (loosely coupled followers):** a mechanism class, such as an `Arm`, now simulates its motor controller's loosely coupled followers too, so a mechanism with two motors on it has the torque of both, where before it only had the leader's. Each motor applies its own output, as its motor controller's simulation limits it, so a follower that isn't driven, such as in open loop, holds the mechanism back. The mechanism iterates the followers right after it steps, so you no longer call their `simIterate()` (it does nothing), each follower's encoder reads the mechanism's simulation, and the battery simulation counts their current once. Without a mechanism class, each motor still simulates its own load.
* **Behavior change (Thrifty Nova):** when the Nova can't run a closed loop, it now runs on the SystemCore, which commands the Nova with voltage, instead of throwing a `SmartMotorControllerConfigurationException`. Every Nova ran its loop this way before v2026.11.05. That is an absolute encoder closing the loop read over a range other than \[0, 1) without continuous wrapping, such as `withWrapping(-0.5, 0.5)` on an arm, and continuous wrapping of an absolute encoder geared to the mechanism. `getClosedLoopControllerPeriod()` (C++ `GetClosedLoopControllerPeriod()`) shows where the loop runs: 1 ms on the Nova, and the closed loop control period (20 ms by default) on the SystemCore, where YAMS also raises its alert that the closed loop runs on the SystemCore. Gains or a feedforward in `SLOT_2` or `SLOT_3`, only one mechanism limit, and the temperature cutoff, closed loop maximum voltage, feedback synchronization threshold or closed loop control period still throw when the Nova runs the loop.
* **Behavior change (Thrifty Nova):** continuous wrapping runs on the Nova over any one rotation range, such as \[-0.5, 0.5) or \[0, 1). The Nova wraps an absolute encoder closing the loop with 1:1 external encoder gearing itself, and YAMS sends each position target folded into \[0, 1); positions are still reported in the configured range. For the motor's own encoder or a quadrature encoder, YAMS sends each target as the equivalent nearest the mechanism (at 3.2 rotations, a target of 0.25 is sent as 3.25), so the Nova's position loop takes the short way. A Nova swerve azimuth seeded from a CANcoder now runs entirely on the Nova, like a SPARK MAX with one.
* **Behavior change (Thrifty Nova):** a feedforward without a motion profile now works, and runs on the Nova. YAMS sends kS, kV, kA and kG to the Nova's motion profile slot either way, and the Nova applies them (with cosine gravity for an arm) with or without a motion profile, in plain position and velocity control too. So a Nova swerve drive motor or flywheel keeps its kS and kV on the Nova, and an arm without a profile keeps its kG. A simulated Nova applies kS, by the sign of the velocity target or of the position error, and kG in position and velocity control too, and kV times the velocity target in velocity control, but not kA, since a plain velocity request has no acceleration target. Nothing about a Nova feedforward needs a profile any more; in v2026.11.05 any feedforward without one threw.
* **Behavior change:** `withClosedLoopRampRate` ramps a closed loop that runs on the SystemCore, an ADRC or an LQR. Such a loop sends the motor controller a voltage, which it treats as open loop, so its own closed loop ramp never applied. The A301 accepts a closed loop ramp rate for this.
* The Commands v3 `advantage_kit` example runs its opmodes with AdvantageKit `27.0.0-alpha-7`'s `LoggedOpModeRobot`: **Angular Velocity Drive** and **Heading Drive** teleops, and a **Score Preload** autonomous.
* The `advantage_kit` and `double_flywheel` examples, in Commands v2 and v3, start with gains that reach their targets in simulation.

### Fixed

* A 2027 robot program no longer stops when it builds its first `SmartMotorControllerConfig`. YAMS reported its usage to the HAL with null names, which WPILib 2027 rejects. Every earlier 2027 release had this.
* **Simulated current:** the `Arm`, `Elevator` and `DCMotor` simulations stepped their supply current filter every time it was read, so a mechanism read by the battery and by telemetry in the same loop reported too much current, and the battery sagged too far. It is filtered once per step. Their stator current read about 0 A after each step, and now reads the current the motors drew.

## 2026.11.05

Released November 5, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.11.05).

### Added

* `SmartMotorControllerConfig.withWrapping(Angle min, Angle max)` (C++ `WithWrapping`): the range an absolute external encoder reads, such as \[-0.5, 0.5) or \[0, 1) rotations, without the closed loop wrapping. The closed loop never takes the mechanism the other way round, so it can be used with soft limits, as on an arm whose encoder reads across zero. `isWrappingContinuous()` (C++ `IsWrappingContinuous()`) says whether the closed loop wraps.
* The Thrifty Nova runs its own closed loop, with ThriftyLib `2027.0.0-alpha-6`: a PID, and a trapezoidal position profile with its feedforward (kS, kV, kA, and kG with cosine gravity for an arm), run on the Nova in its two slots. Its sensor to mechanism ratio puts targets, gains and soft limits in mechanism rotations, the closed loop tolerance is the slot's allowable error, the mechanism limits are the Nova's soft limits, and continuous wrapping over \[0, 1) of an absolute encoder is the Nova's. Live setters update the Nova. An LQR, an exponential profile or a velocity trapezoidal profile still runs on the SystemCore.
* Simulation: a simulated Nova runs its closed loop as the Nova does, since ThriftyLib's simulation ignores PID gains, and a Thrifty CAN encoder and the Nova's data port encoders follow the simulated mechanism.
* `A301Wrapper` (Java `yams.core.motorcontrollers.local`, C++ `yams/motorcontrollers/local/A301Wrapper.hpp`) for the FIRST A301 smart motor, with REVLib `2027.0.0-alpha-9`. The A301 has no closed loop gains, so its closed loop always runs on the SystemCore and commands it with voltage. It has no current limits, ramp rates, voltage compensation, vendor config or followers, and configuring them throws a `SmartMotorControllerConfigurationException`. The gearing given to `withGearing` is from the A301's rotor to the mechanism, so it includes the A301's gearbox (135:1, 64:1, 36:1, 27:1, 13.5:1 or 3:1). Passing the A301 itself to `withExternalEncoder` uses its absolute encoder, on its output shaft. REVLib does not simulate the A301, so in simulation the wrapper drives a `DCMotorSim` of its motor.
* `YMotors.getA301(int numMotors)` and a C++ `yams/motorcontrollers/YMotors.hpp` with `YMotors::A301(int numMotors = 1)`: the A301's motor at 12 V, 13,500 RPM free and 0.078 Nm stalled, with the NEO 550's 100 A stall and 1.4 A free currents until FIRST publishes the A301's. The `A301Wrapper` uses it, so it takes no `DCMotor`.
* Examples, in Commands v2 and v3: `a301_arm`, an arm driven by one A301 through its 64:1 gearbox and a 3:1 chain, with A, B and Y setpoints and an autonomous that tours them.
* Examples, in Commands v2 and v3: `nova_arm`, an arm driven by a NEO on a Thrifty Nova through a 25:1 gearbox, with its position loop, trapezoidal profile and arm feedforward on the Nova, and the same setpoints and autonomous.

### Changed

* **Breaking:** `withExternalEncoderDiscontinuityPoint(...)` (C++ `WithExternalEncoderDiscontinuityPoint`) is removed. An absolute encoder reads the range set with `withWrapping`, or with `withContinuousWrapping`, which also makes the closed loop take the shortest path. Each motor controller accepts only the ranges its encoder can read: a CANcoder or CANdi any range from \[-1, 0) to \[0, 1), a SPARK absolute encoder the same, a SPARK `DetachedEncoder` \[-0.5, 0.5) or \[0, 1), and a Nova closing its loop on the encoder \[0, 1). Anything else throws a `SmartMotorControllerConfigurationException`. Without wrapping, a SPARK or Nova absolute encoder reads \[0, 1) and a CANcoder or CANdi keeps its own range.
* **Breaking:** `getContinuousWrapping()` and `getContinuousWrappingMin()` are now `getWrappingMax()` and `getWrappingMin()` (C++ `GetWrappingMax()`, `GetWrappingMin()`). They return the range whether or not the closed loop wraps.
* **Breaking:** continuous wrapping and soft limits throw a `SmartMotorControllerConfigurationException` in either order, in Java as in C++. Java used to accept soft limits set after continuous wrapping.
* **Breaking (Thrifty Nova):** when the Nova runs the closed loop, a feedforward without a trapezoidal profile, gains or a feedforward in `SLOT_2` or `SLOT_3`, only one mechanism limit, and the temperature cutoff, closed loop maximum voltage, feedback synchronization threshold or closed loop control period throw a `SmartMotorControllerConfigurationException`. Give a mechanism with a feedforward a trapezoidal profile, or a velocity trapezoidal profile for velocity control, which runs on the SystemCore. Gains tuned with the loop on the SystemCore need tuning again on the Nova.
* REVLib `2027.0.0-alpha-9`. A SPARK wraps its position over the mechanism's continuous wrapping range with REVLib's position wrapping input range, for MAXMotion too, and is sent the setpoint as it is. Its absolute encoder's range offset is the middle of the wrapping range.
* The alerts for a discontinuity point without an external encoder (TalonFX, TalonFXS) and for external encoder gearing with a discontinuity point (SPARK) are removed with the option.

### Fixed

* A closed loop on the SystemCore no longer runs away when the mechanism starts at the edge of its continuous wrapping range, such as at 0 with \[0, 1) wrapping: the loop starts its wrapping from the position its motion profile starts from.
* Simulation: a simulated CANdi reports its PWM position in the wrapping range, as a real CANdi does.
* Simulation: a disabled robot's motors get no power. The arm, elevator and DC motor simulations held their last voltage while the robot was disabled, so an arm held itself up instead of falling. They now apply 0 V on every sim step while disabled, for every motor controller.
* A closed loop on the SystemCore no longer jumps when the robot is enabled. While disabled it now follows the mechanism, so it starts from where the mechanism is, not where it was before the mechanism fell. Before its first setpoint, a trapezoidal profile in Java also started from position 0, so the first setpoint after enabling could swing the mechanism the wrong way.
* Simulation: each mechanism's Mechanism2d moves. `Telemetry.log` writes a `Mechanism2d` as it is when logged, and mechanisms only logged theirs when they were made, so the sim GUI showed them standing still. Each visualization update now logs it again.

## 2026.11.03

Released November 3, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.11.03).

### Added

* LQR live tuning: a motor controller configured with an LQR publishes its LQR values under `closedloop/lqr/` in place of kP, kI and kD, at `HIGH` (Java and C++). An arm or elevator gets `qelms/position`, `qelms/velocity`, `relms`, `model/position`, `model/velocity`, `encoder/position` and `maxVoltage`; a flywheel gets `qelms/velocity`, `relms`, `model/velocity`, `encoder/velocity` and `maxVoltage`. They start at the configured values, in degrees and RPM (meters and m/s for an elevator). A change rebuilds the LQR without a jump in its output, and a value the LQR can't use, such as 0, is ignored with a Driver Station warning. `LQRController.retune(config)` does this from code, and `LQRConfig` has getters and checked setters for its values.
* `SmartMotorControllerConfig.withWheelMomentOfInertia(Distance diameter, Mass mass)` (C++ `WithWheelMOI`): a wheel's moment of inertia, as a solid disk (½·m·r²). `withMomentOfInertia(Distance, Mass)` is a rod turning about one end (⅓·m·L²), which is right for arms but gives about 3 times too much for a wheel.
* `SwerveInputStream` telemetry publishes `rotationController/measurement` and `rotationController/setpoint` at `MID`: the heading the rotation controller is given and the heading it turns to, in radians, while the stream aims, holds a heading or translates only, and NaN otherwise. `getRotationControllerMeasurement()` and `getRotationControllerSetpoint()` (C++ `GetRotationControllerMeasurement()` and `GetRotationControllerSetpoint()`) give the same values.
* A `StringTelemetry` primitive (Java `yams.core.telemetry`, C++ `yams/telemetry/StringTelemetry.hpp`), and `isEnabled()` on the Java `DoubleTelemetry`, `BooleanTelemetry` and `StringTelemetry`, as C++ has `IsEnabled()`.
* Examples, in Commands v2 and v3: `double_flywheel`, two `FlyWheel`s in one subsystem tuned with one Live Tuning command, and `lqr_mechanisms`, an LQR arm and an LQR flywheel.

### Changed

* **Flash SPARK MAX and SPARK Flex firmware 27.0.0**, and YAMS now uses REVLib `2027.0.0-alpha-8`. The firmware changes how mechanisms run, so check their gains: followers no longer run about 1% slower than their leader, nor apply voltage compensation twice; kS no longer turns the motor at a velocity setpoint of 0; and a SPARK MAX closing its loop on an absolute or alternate encoder keeps using it after a reboot.
* **Breaking:** the `SwerveDrive` live tuning command is published at `Tuning/<name>/Live Tuning` and named `Live Tuning`, like every other mechanism's. It was at `Mechanisms/<name>/tuning`. Move any dashboard button or code that starts it.
* **Behavior change:** the `SwerveDrive` live tuning command requires the drive (Commands v2 subsystem, Commands v3 mechanism, C++), so the default drive command pauses while it runs and comes back when it ends. Before, the default drive command kept pointing the wheels back where they were.
* **Behavior change:** `runTo(Supplier, tolerance)` on `Arm`, `Pivot`, `Elevator` and `FlyWheel` sends the supplier's value as the setpoint every loop, and ends once the mechanism has been within tolerance of its current value for 0.1 s (Commands v2, Commands v3 and C++). It read the supplier once, when the command was built. Each command has its own debouncer, reset when it starts.
* **Behavior change:** `SmartMotorControllerConfig.clone()` is a deep copy. Motors cloned from one config shared its PID controller, feedforward and LQR, so live tuning one changed the others.
* **Behavior change:** setting a setpoint starts a stopped closed loop that runs on the SystemCore (an LQR or exponential profile), from every setpoint command, the `SmartMechanism` setpoint setters and Live Tuning. A loop that is already running carries on, where the setters used to restart it on every call and reset an LQR's state estimate or a PID's integral each loop.
* **Breaking (C++):** `LQRConfig::GetLoop()` returns a `LQRConfig::LoopBundle`, which owns the plant, LQR, Kalman filter and loop together.
* `LQRConfig.withAggressiveness` is deprecated: it never changed anything. Use qelms and relms.
* `SwerveInputStream` telemetry is built on telemetry fields like the rest of YAMS, and its topics carry units. The NetworkTables keys are unchanged.

### Fixed

* An LQR or exponential profile mechanism stopped by a duty cycle or voltage command, such as a coasting default command, stayed stopped when given a setpoint. An LQR flywheel stayed at 0 RPM in Live Tuning.
* **C++:** the LQR read freed memory on every predict and correct, so in simulation an LQR elevator never braked.
* Simulation: a Kraken X44 on a TalonFX was simulated as a Kraken X60, so above an X60's free speed Phoenix held its output and an elevator ran into its hard stop. The TalonFX now recognizes a Kraken X44 from its `DCMotor` (Java and C++).
* A TalonFX's loosely coupled followers switch closed loop slot with it, as the TalonFXS, SPARK and Thrifty Nova already did.
* A TalonFX or TalonFXS vendor config wins over the device's saved settings. With `withResetPreviousConfig(false)`, the Java wrappers replaced the vendor config's settings with the ones saved on the device.
* `ExponentialProfilePIDController.createElevatorConstraints` returned NaN or infinite constraints. It now returns meters, like `withExponentialProfile(Voltage, DCMotor, Mass, Distance)`.
* `withExternalEncoderGearing` warned the wrong way round. It now warns when the encoder turns more than the mechanism and can wrap (Java and C++).
* `LQRConfig.withMeasurementDelay` was stored and never used. It now compensates the LQR for the delay.
* `SwerveInputStream` live tuning applied a value left on the dashboard by an earlier telemetry with the same name to the stream. The stream's own value replaces it.

## 2026.10.22

Released October 22, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.10.22).

### Added

* Live tuning can drive a motor at a voltage: `voltage/enabled` and `voltage/setpoint` under `Tuning/<mechanism>/<motor>/`, at `HIGH` (Java and C++). Turning it on stops the closed loop and starts at 0 V; the voltage is sent to the motor and its loosely coupled followers every loop; turning it off, or ending the Live Tuning command, starts the closed loop again and sends the setpoint it held.
* `SmartMotorController.setupLiveTuning()` (C++ `SetupLiveTuning()`): registers the Live Tuning command when the motor's telemetry is at `HIGH`, once.
* The release notes start with a table of the example robot projects attached to the release, ready to download with that release's vendordep, then how to get started.

### Changed

* Every Commands v3 example except `advantage_kit` runs each mechanism's simulation in its own `Notifier`.
* Simulation: a SPARK reports its applied output times its bus voltage, as on the robot.

### Fixed

* Live Tuning never showed up for an `Arm`, `Elevator`, `Pivot` or `FlyWheel` with `.withTelemetry(...)` in its mechanism config, nor for `DifferentialMechanism` and `DoubleJointedArm` motors added with `addMotorController` (Java and C++).
* `BatterySim` is thread safe. Mechanisms simulated in their own `Notifier`s threw `ConcurrentModificationException` in Java, and crashed in C++.

## 2026.10.19

Pre-release, October 19, 2026 ([release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.10.19)).

### Added

* `getPose3d()` on every mechanism (C++ `GetPose3d()`): its `Pose3d` relative to the robot, to publish as a component of an AdvantageScope robot model. It is worked out from the motor's position, so it works on the robot as well as in simulation: an arm's or pivot's axis turned to its angle, an elevator's carriage at its height, or a flywheel's axle turned by its position. `DoubleJointedArm.getUpperPose3d()` gives the upper arm's. See [How do I show my mechanisms on a 3D robot model?](https://yams.yassrobotics.com/documentation/how-to-simulation-and-logging/how-do-i-show-my-mechanisms-on-a-3d-robot-model).
* `MechanismPositionConfig.withRelativePosition(Transform3d)`, which also sets how the mechanism is turned where it is mounted, and `withParent(Supplier<Pose3d>)`, which mounts it on another mechanism, such as an arm on `elevator::getPose3d`. `getMountPose()`, `getRotatedPose(Angle)` and `getTranslatedPose(Distance, Angle)` do the math for parts YAMS has no mechanism for.
* **C++:** `ArmConfig`, `ElevatorConfig`, `PivotConfig` and `FlyWheelConfig` take a `MechanismPositionConfig` with `WithMechanismPositionConfig(...)`.
* `ElevatorConfig.withStages(Distance... stageHeights)` (C++ `WithStages(0.6_m, 0.6_m, 0.5_m)`): how far each of the elevator's moving stages travels relative to the stage below it, one height for each stage, the carriage last. `Elevator.getCascadePose3d()` and `getContinuousPose3d(boolean carriageFirst)` (C++ `GetCascadePose3d()` and `GetContinuousPose3d(bool carriageFirst)`) then give each stage's pose, from the stage nearest the fixed stage to the carriage: a cascade elevator's stages all move at once, each by its share of the carriage's height, and a continuous elevator's stages move one at a time, the carriage first or last. Both need 2 or more stages. The stage heights must add up to the elevator's maximum height, or the `Elevator` throws an `ElevatorConfigurationException`, and without hard limits they give the elevator's height, from 0 to their total (`getStageHeightTotal()`). See [Elevator stages](https://yams.yassrobotics.com/documentation/how-to-simulation-and-logging/how-do-i-show-my-mechanisms-on-a-3d-robot-model#elevator-stages).
* `YMotors.getThriftyPulsar(int)`: a `DCMotor` of the Thrifty Pulsar 775 from Thrifty's specs, since WPILib has none. `TalonFXSWrapper` drives it as `NEO_JST`.
* Swerve live tuning can drive the modules at a voltage, without their closed loops, to characterize them: `modules/drive/voltage/enabled` and `modules/drive/voltage/setpoint` while drive tuning is on, and `modules/azimuth/voltage/enabled` and `modules/azimuth/voltage/setpoint` while azimuth tuning is on, under `Tuning/<name>/`. A closed loop that runs on the SystemCore is stopped while its voltage is on. The YAGSL config generator's Live mode uses these for its **Auto** gains. See [Characterize the modules](https://yams.yassrobotics.com/documentation/understanding/live-tuning#characterize-the-modules).
* `SmartMotorController.getClosedLoopControllerPeriod()` (C++ `GetClosedLoopControllerPeriod()`): how often the closed loop runs, 1 ms on the motor controller or the closed loop control period on the SystemCore. Published as `closedloop/period` at `HIGH`, or with `withClosedLoopPeriod()` (`WithClosedLoopPeriod()`).
* `SwerveModule.getCouplingVelocity()` (C++ `GetCouplingVelocity()`) is public, and the C++ `SwerveDrive` has `ApplyTuningValues()`, as Java does.
* SPARK and TalonFX feedback and feedforward gains are checked with the vendor's status code when they change, retrying for up to 15 ms, and raise an alert if they still aren't applied. Only the SPARK's closed loop or feedforward config, or the TalonFX's slot, is sent, to use less of the CAN bus.

### Changed

* **Breaking:** the mechanisms' `getRelativeMechanismPosition()` is replaced by `getPose3d()` (C++: `GetRelativeMechanismPosition()` by `GetPose3d()`). The old method was read from the simulation's `Mechanism2d`, threw a `NullPointerException` on the robot, gave degrees to `Rotation3d` as radians, and turned the wrong way. `SmartMotorController.getRelativeMechanismPosition()`, which reads the relative encoder, is unchanged.
* **Breaking (C++):** `MechanismPositionConfig` is now the class that places a mechanism in 3D, as in Java. It was a named setpoint class that was declared but never defined.
* A `Pivot`'s pose turns in its movement plane: XY, like a turret, unless it is given a `MechanismPositionConfig`, which is XZ unless set otherwise. The C++ `PivotConfig` defaults to XY too, as the Java one does.
* **Behavior change:** stopping a `SmartMotorController`'s closed loop, as a mechanism does before driving it at a duty cycle or voltage, also stops its loosely coupled followers' closed loops and sets them to a duty cycle of 0. Starting it starts theirs (Java and C++).
* `TalonFXSWrapper` drives a NEO Vortex as `VORTEX_JST` rather than `NEO_JST`, as the C++ wrapper and YAMG already did.
* **Behavior change:** swerve live tuning applies only the values that changed, and sends the setpoints again when a tuning mode turns on or off. A tuning mode that is on turns off the ones it outranks (auto-align, then drive tuning, then azimuth tuning). Ending the tuning command turns every mode off and holds the wheels where they are.
* **Breaking (C++):** the swerve tuning command is published at `Mechanisms/<name>/tuning`, as in Java, rather than `Mechanisms/<name>/tuning/driveToPose`.

### Removed

* **Breaking:** `SmartMechanism.getRelativeMechanismPosition()` (Java) and `GetRelativeMechanismPosition()` on the C++ `Arm`, `Elevator`, `Pivot` and `FlyWheel`. Use `getPose3d()`.

### Fixed

* A loosely coupled follower whose closed loop runs on the SystemCore (a Thrifty Nova, a SPARK with an exponential profile, or any motor controller with an LQR) kept driving to its last setpoint after its leader stopped, including after a duty cycle of 0: its closed loop set its output again on the next loop.
* Simulation: a `Pivot` stops at its hard stops. It kept its speed when put back on a stop, and flew past again on the next loop. A pivot that wraps continuously (`withContinuousWrapping`) has no hard stops, and turns on past where it wraps (Java and C++).
* Simulation: a `Pivot` starts at its starting position. Its simulation started at 0, away from its encoder, so it was stopped at hard stops it had not reached (Java and C++).
* Simulation: a TalonFX or TalonFXS keeps an encoder position it is set to. It set the position before moving its simulated rotor, so a Talon at 60 degrees set to 0 read -60 (Java).
* `TalonFXSWrapper` threw `IllegalArgumentException("Unknown motor for TalonFXS...")` for a Thrifty Pulsar.
* C++ `cpptest` example: `projectYear` is `2027_alpha7`, to match its GradleRIO.

## 2026.10.15

Released October 15, 2026. Download the example robot projects from [the release](https://github.com/Yet-Another-Software-Suite/YAMS/releases/tag/v2026.10.15).

### Added

* `SwerveInputStream` aim options, carried over from YAGSL 2026: `withAimLookahead(Time)`, `withAimFeedforward(SimpleMotorFeedforward)`, `withAimHeadingOffset(Rotation2d)`, `isAimLocked(Angle)` and the `aimLock(Angle)` Trigger, `getTargetVector(Pose2d)` and `calculateAngularVelocity(Angle)`. Aiming without them behaves as before.
* `SwerveDrive.setAzimuthAngles(Rotation2d)`: turns every module to one angle without driving it. Azimuth live tuning uses it, so it turns the modules again.
* **C++:** the swerve drive matches the Java one: the 3D gyro and gyro heading axis, anti-tipping, module speed desaturation, the coupling ratio, feedforward forces and the default velocity deadband.
* Commands v2 `SwerveDrive.driveToPose(Pose2d, Distance, Angle)`, which ends once the robot is within the tolerances, and `isNear(Pose2d, Distance, Angle)` and `stop()`. These were only on the Commands v3 `SwerveDrive`; `isNear` and `stop` now live on the shared `SwerveDrive`.
* `SmartMechanism.getMeasurementSetpoint()`: the mechanism's position setpoint as a `Distance`, converted with the mechanism circumference. It is empty when there is no position setpoint or no circumference is configured.
* `EasyCRT.solve(Angle, Angle)` / `EasyCRT::Solve(...)`: solve for travel centered on zero, with a `solve(a, b, minAngle, maxAngle)` overload for other travel.
* `EasyCRT.getUniqueRange()` / `EasyCRT::GetUniqueRange()`: how far the shared gear can turn before the two encoder readings repeat.
* A `Turret` that seeds its motor from `EasyCRT` at startup, with a unit test, in the `BroncBotz_2026` examples for Commands v2 and Commands v3.

### Changed

* **Behavior change:** an `ArmFeedforward`'s kV and kA are per rotation everywhere. The closed loop on the SystemCore (such as the Thrifty Nova's) read them per radian. If you tuned an arm's kV or kA on the SystemCore with 2026.10.9, multiply them by 2π to keep the same behavior.
* **Breaking:** `SwerveModule.lock(Rotation2d)` is now `setAzimuthAngle(Rotation2d)`. It turns the wheel without driving it.
* The external encoder telemetry is only published when there is an external encoder. `externalencoder/position` and `externalencoder/velocity` keep their names.
* The vendordep JSONs require the matching WPILib command library (`command2` or `command3`).
* **Behavior change:** the Commands v2 `SwerveDrive.driveToPose(Pose2d)` now stops the drive when it ends or is interrupted, as the Commands v3 one already did. It used to leave the last commanded speeds in place.
* **Behavior change:** `withTemperatureCutoff` now follows where the closed loop runs, because the cutoff is only checked by the closed loop on the SystemCore (Java and C++). On a SPARK MAX or SPARK Flex it needs an exponential profile or an LQR controller: a trapezoidal profile with a cutoff used to be accepted and silently ignored, and now throws at startup. On a TalonFX or TalonFXS it is now accepted with an LQR controller, where it used to always throw.
* **Breaking:** `EasyCRT` is now built from three tooth counts and solves from two readings (Java and C++). The match tolerance is derived from the tooth counts, and a reading pair that does not agree returns an empty result.
* **Breaking (Java):** `DifferentialMechanism.set(...)` now takes `(tilt, twist)` like every other method, and mixes them with the same signs as `setPosition` (left = tilt + twist, right = tilt - twist). It took `(twist, tilt)` and drove the twist the opposite way. Swap the arguments if you called it as `(twist, tilt)`.
* `EasyCRTConfigGuide.md` is replaced by `EasyCRTGuide.md`, which explains how to choose tooth counts, zero and invert the encoders, handle gear trains, and check your margin yourself.
* `MIGRATION.md` now covers the whole 2026 to 2027 upgrade.

### Removed

* **Breaking:** `EasyCRTConfig`, including its gearing helpers, offset and inversion setters, match tolerance, and gear recommender, and the `EasyCRT` status, error and iteration getters.

### Fixed

* Simulated SPARKs follow their profile and feedforward. REVLib's simulation does not run MAXMotion, and works out kCos from the setpoint instead of the position, so in simulation a SPARK with a trapezoidal profile runs the profile on the SystemCore. On the robot it still uses MAXMotion. See [wpilibsuite/SystemcoreTesting#425](https://github.com/wpilibsuite/SystemcoreTesting/issues/425).
* Simulated SPARKs start at their starting position, and report their real voltage. Both were off by the gear ratio.
* Every motor controller gives the same voltage for the same kP, kI, kD, kS, kV, kA and kG in YAMS units, from every closed loop slot. This fixed an elevator feedforward under velocity control on the SystemCore, which left out kV.
* **C++:** swerve fixes the Java swerve already had, and swerve azimuth live tuning.
* `ExponentialProfilePIDController.createConstraints(maxVolts, maxVelocity, maxAcceleration)` passed kV and kA as state space terms, which gave an unstable profile with a negative maximum velocity. It now builds the same constraints as `SmartMotorControllerConfig.withExponentialProfile(Voltage, AngularVelocity, AngularAcceleration)`.
* Simulation: `Arm`, `Elevator`, `Pivot`, `FlyWheel`, `DoubleJointedArm` and `DifferentialMechanism` no longer replace the shared `BatterySim` voltage with one computed from their own current draw alone. The simulated battery now sags from every mechanism's combined draw, and `enableDischarge(...)` is respected, no matter which mechanism iterates last (Java and C++).
* `simple_robot-2027` examples: the simulated gyro yaw was set in radians instead of degrees, and the elevators were missing the carriage weight and starting position that `Elevator` requires in simulation.
* C++ `cpptest` example: the field-relative `SwerveInputStream` output is converted to robot-relative before `SwerveDrive::Drive`.
* `GearBox` Javadoc: `fromTeeth(...)` is a train of meshing gears, so `fromTeeth(12, 60, 12, 60)` is 5:1, not 25:1.

## 2026.10.9

### Added

* `NovaWrapper` (Java `yams.core.motorcontrollers.local.NovaWrapper`, C++ `yams/motorcontrollers/local/NovaWrapper.hpp`): a `SmartMotorController` for the Thrifty Nova, with simulation, telemetry, and alert support. Requires the ThriftyLib 2027 vendordep.
* `SmartMotorController.getRelativeMechanismPosition()` / `getRelativeMechanismVelocity()` (Java) and `GetRelativeMechanismPosition()` / `GetRelativeMechanismVelocity()` (C++): read only the relative (motor) encoder, even when an external encoder is configured. `getMechanismPosition()` / `getMechanismVelocity()` still prefer the external encoder.
* **C++:** `SmartMotorController`, `SmartMotorControllerConfig`, simulation, telemetry, `SparkWrapper`, `TalonFXWrapper`, and `TalonFXSWrapper` now match the Java API.

### Changed

* **Breaking:** `getExternalEncoderPosition()` / `getExternalEncoderVelocity()` renamed to `getExternalEncoderMechanismPosition()` / `getExternalEncoderMechanismVelocity()` (C++: `GetExternalEncoderMechanismPosition()` / `GetExternalEncoderMechanismVelocity()`).
* Alerts now refer to the SystemCore instead of the RIO.
*   Maven artifacts (Java and C++) and the vendordep JSONs are now served from `https://cdn.yassrobotics.com/`, so networks that block `github.io` can still install and update YAMS. Reinstall the vendordep from the new URL:

    | Command framework | Vendordep URL                                      |
    | ----------------- | -------------------------------------------------- |
    | Commands v2       | `https://cdn.yassrobotics.com/yams_commands2.json` |
    | Commands v3       | `https://cdn.yassrobotics.com/yams_commands3.json` |

## 2026.10.8

### Added

* `SwerveInputStream.withTelemetry(String, TelemetryVerbosity)` (Java) / `SwerveInputStream::WithTelemetry(...)` (C++): publishes the stream's drive mode (`LOW`), its configuration (`MID`), and editable copies of the configuration plus a `Live Tuning` command (`HIGH`). The stream publishes every time it is read, so there is nothing to call each loop. Access it with `getTelemetry()` / `GetTelemetry()`.

{% hint style="info" %}
Call `withTelemetry(...)` last, after the rest of the stream configuration, so the dashboard starts with the configured values. A `clone()` of a stream has no telemetry.
{% endhint %}

## 2026.10.5

### Added

* `GyroAxis` enum (`yams.core.mechanisms.config.enums`: `YAW` default, `ROLL`, `PITCH`) and `SwerveDriveConfig.withGyroHeadingAxis(GyroAxis)` (Java): use a gyro mounted on its side as the heading source. The gyro attitude is rotated into the robot frame, so the heading keeps its full range and anti tipping reads the robot's tilt.
* `SwerveModule.lock(Rotation2d)` and `SwerveModuleConfig.getLockedState(...)` (Java).

### Changed

* **Breaking (Java):** `SwerveDriveConfig.getGyroAngle()` and `SwerveDrive.getGyroAngle()` are removed. Use `getGyroRotation3d()`. The heading now wraps at +/-180 degrees instead of counting continuously.
* Vendordeps now declare `requires` and `conflictsWith`, so the Commands v2 and Commands v3 variants of YAMS cannot be installed together.

### Fixed

* **Java:** `SwerveDrive.lockPose()` now turns the wheels into the X pattern. Previously the module velocity deadband replaced the zero speed states, so the wheels held their current angle.
* **Java:** Simulated SPARKs now close their loop at 1 kHz like a real SPARK (and like simulated Talons), instead of once per 20 ms simulation period. Integral and derivative gains are no longer scaled by the loop period in simulation. Real robot gain conversion is unchanged.

## 2026.10.3

YAMS now targets **WPILib 2027** (`2027_alpha7`) and the SystemCore. Robot code built against earlier releases needs to be migrated; see **Upgrading from 2026 to 2027** in the [YAMS documentation](https://yagsl.gitbook.io/yams/documentation).

### Added

* Two vendordeps, one per WPILib command framework. Install exactly one:
  * **Commands v2:** `yams_commands2.json`, with command bindings in `yams.commands2.*`.
  * **Commands v3:** `yams_commands3.json`, with command bindings in `yams.commands3.*` (Java only, since WPILib has no C++ Commands v3).
* An offline installer, `<version>-YAMS-Offline.zip`, attached to each GitHub release. Extract it into your WPILib `2027_alpha7` folder, then install the vendordep offline.
* Example projects for both command frameworks under `examples/commands2` and `examples/commands3`.

### Changed

* **Breaking:** WPILib 2027 packages (`org.wpilib.*` in Java, `wpi::` in C++), Java 25, Phoenix 6 2027, and REVLib 2027.
* **Breaking (Java):** packages split into `yams.core.*` (mechanism and motor controller state, simulation, telemetry, configs, wrappers, gearing, math, units) and `yams.commands2.*` / `yams.commands3.*` (subsystem bound mechanisms, `SmartMotorControllerConfig`, `SwerveDriveConfig`, `SwerveDrive`, `SwerveInputStream`, and their `Command`/`Trigger` factories). Robot code imports mechanisms and those configs from the command framework package.
* **Breaking (Java):** the `isNear(...)` `Trigger` factory is renamed `near(...)`, matching `gte()`, `lte()`, `between()`, `max()`, and `min()`. The boolean `isNear(...)` check is unchanged.
* Live tuning registers itself when telemetry is `TelemetryVerbosity.HIGH`. In other cases, opt in with `SmartMotorControllerConfig.setupLiveTuning()`.
* **Breaking:** `withIdleMode(MotorMode)` is renamed `withZeroPower(MotorMode)` (C++: `WithZeroPower`).

## 2026.9.16

### Changed

* **Breaking:** `SmartMotorControllerConfig.getZeroOffset()` is renamed `getExternalEncoderZeroOffset()`, matching the existing `withExternalEncoderZeroOffset(...)` setter.

### Fixed

* `TalonFXWrapper` no longer applies the zero offset to the rotor as well, which double applied the offset.

## 2026.9.10

### Added

* `SmartMotorController.setVelocity(AngularVelocity, Force)` / `setVelocity(LinearVelocity, Force)` and `SwerveDrive.setRobotRelativeChassisSpeeds(..., Force[])`: pass a feedforward force (for example from a path follower) straight through to the drive motors. `SwerveModule` gained the matching overloads.
* `SwerveDriveConfig.clone()`.
* `SmartMotorControllerConfig.withSimulationPeriod(Time)` (C++: `WithSimulationPeriod`).
* `SimSupplier.getStatorCurrent()` / `getSupplyCurrent()`.
* The gyro angular velocity is derived from the heading when no gyro velocity supplier is set, so angular velocity scaling works without one.

### Fixed

* The simulated swerve pose was integrated twice, so it moved robot relative instead of field relative.

## 2026.9.2

### Added

* Advanced telemetry for `SwerveDrive` and `SwerveModule`: `SwerveDriveConfig.withTelemetry(String, TelemetryVerbosity)` / `withTelemetry(String, SwerveDriveTelemetryConfig)` and `SwerveModuleConfig.withTelemetry(String, SwerveModuleTelemetryConfig)`, with more live tunable fields.
* `SwerveDrive.setTranslationPID(PIDController)`, `setRotationPID(PIDController)`, `driveToPoseSetpoint(Pose2d)`, and `getModules()`.
* `SwerveModule.getRawAbsoluteEncoderAngle()`.

### Changed

* `SwerveDrive.resetAzimuthPID()` is renamed `resetRotationPID()`.
* `SwerveInputStream` takes its maximum velocities from the `SwerveDrive`.

### Removed

* **Breaking:** `SwerveDriveConfig.withDataLogName(...)` and `SwerveModuleConfig.withDataLogName(...)`. Use the `withTelemetry(...)` overloads above.
* **Breaking:** drive to pose in `SwerveInputStream`, including `atTargetPose(double)`. Use `SwerveDrive.driveToPoseSetpoint(Pose2d)`.

### Fixed

* The absolute encoder supplier is zeroed when an absolute encoder is supplied to `SwerveModuleConfig`.

## 2026.8.26

### Added

* `SwerveDrive.getSimPose()` (Java) / `SwerveDrive::GetSimPose()` (C++) — a ground-truth simulated pose that assumes every module reached its last-commanded `SwerveModuleState` perfectly. It's integrated in `simIterate()`/`SimIterate()` from the desired (not measured) module states via a `Twist2d`/`frc::Twist2d`, independent of the noisy, gyro/odometry-fused pose returned by `getPose()`/`GetPose()`. `resetOdometry(...)`/`ResetOdometry(...)` also snaps it to the given pose, keeping it in sync with the fused estimate. Intended as a "known truth" source for simulated vision systems.
* `SwerveDrive.getRobotRelativeChassisSpeedsFromState(SwerveModuleState[])` (Java) / `SwerveDrive::GetRobotRelativeChassisSpeedsFromState(...)` (C++) — converts an arbitrary set of module states back into robot-relative chassis speeds.
* `SwerveDrive.getDesiredModuleStates()` (Java) / `SwerveDrive::GetDesiredModuleStates()` (C++) — returns the last-commanded module states, mirroring the existing `getDesiredChassisSpeeds()`/`GetDesiredChassisSpeeds()`.

### Removed

* **Breaking:** `SwerveDrive.getPoseEstimator()` (Java) / `SwerveDrive::GetPoseEstimator()` (C++) — direct access to the underlying `SwerveDrivePoseEstimator` was too easy to misuse (calling `update(...)`/`resetPosition(...)`/`resetPose(...)` on it directly corrupts the pose estimate or desyncs it from the drive's gyro offset). Use `getPose()`, `resetOdometry(Pose2d)`, and `addVisionMeasurement(...)` instead.

## 2026.8.20

### Added

* `BatterySim` (Java & C++) now models a single, shared robot battery in simulation. Every simulated `SmartMotorController` — via its `SimSupplier` (`ArmSimSupplier`, `ElevatorSimSupplier`, `DCMotorSimSupplier`) or hardware wrapper (`SparkWrapper`, `TalonFXWrapper`, `TalonFXSWrapper`) — reports its own current draw into one combined loaded-voltage calculation, instead of each mechanism computing `RoboRioSim` voltage sag from only its own current draw and overwriting the others.
* `BatterySim.enableDischarge(double, Voltage, Resistance)` (Java) / `BatterySim::EnableDischarge(double, units::volt_t, units::ohm_t)` (C++) — optional realistic battery discharge simulation. Current draw is integrated into amp-hours consumed over time, sagging the open-circuit voltage and raising internal resistance as the battery empties, following a typical FRC sealed lead-acid discharge curve. Also added `disableDischarge()`/`DisableDischarge()`, `resetDischarge()`/`ResetDischarge()`, and `getStateOfCharge()`/`GetStateOfCharge()`.
* `BatterySim.replaceSOCInterpolation(InterpolatingDoubleTreeMap)` (Java) / `BatterySim::ReplaceSOCInterpolation(const std::map<double, double>&)` (C++) — swap in a custom state-of-charge → voltage discharge curve, for modeling a well-used competition battery, a different chemistry, or a curve measured from a real battery on a load tester.

### Changed

* **C++:** `Arm`, `Elevator`, `Pivot`, and `FlyWheel` no longer call `RoboRioSim::SetVInVoltage(...)` directly from their own current draw. Voltage now comes from the shared `BatterySim`, so combined load across multiple mechanisms sags the simulated bus voltage realistically instead of only reflecting whichever mechanism updated last.

{% hint style="info" %}
Discharge simulation is off by default and only affects simulation — on a real robot, `RobotController.getBatteryVoltage()`/`frc::RobotController::GetBatteryVoltage()` always reflects actual hardware.
{% endhint %}

## 2026.8.18

### Added

* `SwerveDriveConfig.getDiscretization()` — returns the discretization time used by `optimizeRobotRelativeChassisSpeeds(...)`, automatically preferring the simulation-specific override from `withSimDiscretizationTime(...)` when running in simulation.

## 2026.8.17

### Added

* `SmartMotorControllerTelemetryConfig.withCustom(DoubleTelemetryField, boolean)` / `withCustom(BooleanTelemetryField, boolean)` (Java, plus array overloads) — an escape hatch to enable or disable a telemetry field directly when there isn't a dedicated `with*()` setter for it yet.

### Changed

* Telemetry now only publishes fields that are actually enabled, instead of publishing every registered field regardless of its enabled state.
* **Java & C++:** Swerve module telemetry now nests each module under `swerve/modules/<name>/...` instead of `swerve/<name>/...`, removing a double `/` that showed up in some NetworkTables/DataLog paths.

### Fixed

* **Java & C++:** Cosine compensation in `SwerveModuleConfig` now reads the azimuth angle once per control cycle and reuses that same value for both state optimization and the cosine scalar, instead of re-querying the absolute encoder separately for each step. Previously those two reads could disagree, which showed up as skewed drive speed or the wheel losing its commanded direction right around a 180° heading error.

## 2026.8.16

### Added

* `MechanismTelemetry` is now the primary coordinator of all mechanism telemetry, and gained explicit DataLog support alongside NetworkTables.
* `SwerveDriveConfig.withDataLogName(String)`/`getDataLogName()` — logs the drive's pose, gyro, and chassis speeds/module states to a WPILib DataLog under the given name, in addition to NetworkTables.
* `SwerveModuleConfig.withDataLogName(String)`/`getDataLogName()` — logs that module's absolute encoder angle to a WPILib DataLog under the given name, in addition to NetworkTables.

### Changed

* `SwerveModule` now shares a single `MechanismTelemetry` instance (and loop timer) across its drive and azimuth motors instead of each motor re-running its own telemetry setup.

{% hint style="info" %}
Neither `SwerveDriveConfig.withDataLogName(...)` nor `SwerveModuleConfig.withDataLogName(...)` cascades down to a module's drive/azimuth motors. Set a DataLog name on each motor's own `SmartMotorControllerConfig`/`SmartMotorControllerTelemetryConfig` for per-motor logging.
{% endhint %}

## 2026.7.10

### Added

* **C++:** `SmartMotorControllerConfig::WithFeedforward(...)` — a single overloaded setter that takes a WPILib feedforward object directly (`frc::ArmFeedforward`, `frc::ElevatorFeedforward`, or `frc::SimpleMotorFeedforward<units::turns>`), replacing the setters removed below.

### Removed

* **Breaking (C++):** `SmartMotorControllerConfig` removed `WithArmFeedforward`, `WithElevatorFeedforward`, `WithSimpleFeedforward`, and the standalone `WithKp`/`WithKi`/`WithKd`/`WithKs`/`WithKv`/`WithKa`/`WithKg` setters. See the migration note above.

## 2026.7.09

### Added

* `SwerveModule` now publishes the azimuth absolute encoder angle to telemetry.
* Added `SwerveModuleConfig.getAbsoluteEncoderSupplier()` and `getRawAbsoluteEncoderAngle()` (Java) to expose the absolute encoder reading directly, including a variant with the configured zero offset stripped out. Swerve telemetry now reports this raw, offset-free angle.

### Changed

* `SwerveModule.setSwerveModuleState(SwerveModuleState)` now returns the optimized `SwerveModuleState` instead of `void`.

### Fixed

* `SwerveDrive`'s desired module states now reflect the actual post-optimization `SwerveModuleState` returned by `setSwerveModuleState(...)` above, instead of the pre-optimization input — fixes desired-state telemetry/logging showing the wrong angle after a flip.

## 2026.7.06

### Fixed

* Fixed `SwerveModuleConfig`'s state optimizer entering an infinite re-optimization loop right at a flip boundary. The flip decision now debounces against the last _commanded_ angle instead of being re-derived from the live absolute encoder every loop.

## 2026.6.25.1

### Added

* `SmartMotorControllerCommandRegistry` now throws `IllegalStateException` when two different `Subsystem` instances register commands under the same name, and adds `removeCommands(Subsystem)` to clear a subsystem's registered commands (e.g. between tests).

## 2026.6.25

### Added

* Added a full C++ port of YAMS: mechanisms (`Arm`, `Elevator`, `Pivot`, `FlyWheel`, `DoubleFlyWheel`, `DifferentialDrive`), motor controller wrappers (Spark, TalonFX, TalonFXS), telemetry, units, and unit tests, mirroring the existing Java API.
* Added `SwerveDrive` and `SwerveInputStream` (new in both Java and C++).
* Added Doxygen documentation generation for the C++ API.

### Removed

* **Breaking:** Removed ThriftyNova motor controller support (`NovaWrapper`) from the Java API; there is no direct replacement.

### Changed

* Adopted `wpiformat` for consistent Java/C++ formatting across the codebase.

## 2026.1.17

* Empirical Method for CRT by [@mjjbycicle](https://github.com/mjjbycicle) in [#68](https://github.com/Yet-Another-Software-Suite/YAMS/pull/68)
* SmartMotorController Control Request Fix by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#69](https://github.com/Yet-Another-Software-Suite/YAMS/pull/69)
* Fixed the DCMotorSim to work with the correct units by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#72](https://github.com/Yet-Another-Software-Suite/YAMS/pull/72)
* Swerve drive sysId by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#73](https://github.com/Yet-Another-Software-Suite/YAMS/pull/73)
* Inversions shouldn't apply in Simulation by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#74](https://github.com/Yet-Another-Software-Suite/YAMS/pull/74)
* MaybeCRT by [@alexalex02](https://github.com/alexalex02) in [#70](https://github.com/Yet-Another-Software-Suite/YAMS/pull/70)
* Kitbot added by [@nobodyhere4](https://github.com/nobodyhere4) in [#62](https://github.com/Yet-Another-Software-Suite/YAMS/pull/62)

## 2026.1.16

* Fixed non-profiled Closed Loop Control in TalonFXS, TalonFX, and SparkMax.
* Added `Conversion` for ThriftyNova encoder readings to account for encoder ticks.
* Fixed `SwerveDrive` template.
* Added Shoot on the move example.
* [Added Hood + Turret example ](https://github.com/Yet-Another-Software-Suite/YAMS/pull/63)(by 9738)

## 2026.1.4

* Added Imperial `YUnits.PoundSquareInches` `YUnits.PoundsSquareFeet`
* Deprecated `withMOI(double)` in favor of `withMOI(MomentOfInertia)`.
* Added `withMOI(MomentOfInertia)`

## 2026.1.2

* Added `SmartMotorControllerConfig().clone()` to allow for clonable configs.
* Added `MechanismConfig.withSmartMotorController()` for all mechanism and null param constructors
* Added `SensorData.set()` overloads
* 'Trapazoidal' Spelling Error Correction by [@akhaled247](https://github.com/akhaled247) in [#56](https://github.com/Yet-Another-Software-Suite/YAMS/pull/56)
* REV 2026 with Feedforwards and PIDs by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#55](https://github.com/Yet-Another-Software-Suite/YAMS/pull/55)
* Added the option to loosely coupled followers. by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#58](https://github.com/Yet-Another-Software-Suite/YAMS/pull/58)
* Allow for independently created config classes for constants files. by [@thenetworkgrinch](https://github.com/thenetworkgrinch) in [#59](https://github.com/Yet-Another-Software-Suite/YAMS/pull/59)
* Changed `SwerveInputStream.copy()` to `SwerveInputStream.clone()`
* Modified `SmartMotorControllerConfig` to allow for creation without a `Subsystem` but still require a subsystem eventually.
* Modified `SmartMotorController` telemetry to only create Live Tuning and other useful commands if tunable setpoints are not enabled.
* Added `MechanismGearing(double...)` overload
* Added DifferentialDrive example.

## 2025.12.22

* Added the ability to use multiple SMC's with Live Tuning in one subsystem
* Added AdvantageKit example
* Modified `SwerveDrive` to support AdvantageKit better.
* Updated Telemetry Setup for standalone SMC's to be automatic.
* Added Units to Telemetry Fields.
* Added DoubleFlyWheel example.
* Fixed ElevatorConfig comments saying "length" instead of "angle" [#54](https://github.com/Yet-Another-Software-Suite/YAMS/pull/54)
* Added support for user moment of inertia in ExponentialProfilePIDController [#50](https://github.com/Yet-Another-Software-Suite/YAMS/pull/50)
* Improved Exponential Elevator Examples [#46](https://github.com/Yet-Another-Software-Suite/YAMS/pull/46)
* Added ExponentialProfile.
* Add MapleSim Integration to SwerveDrive by [@konnorreynolds](https://github.com/konnorreynolds) in [#49](https://github.com/Yet-Another-Software-Suite/YAMS/pull/49)

## 2025.11.05

* Renamed `.withExternalGearing` to `.withExternalEncoderGearing`.
* Added several new easy commands, like "Up", "Down", and "ZeroEncoder".
* Moved commands to "SmartDashboard/Mechanisms/MECHNAME/MOTORNAME/Commands/".
* Fixed immutable SensorData fields.
* Fixed `SparkWrapper.setEncoderPosition` to set the sim position correctly.

## 2025.11.02

* Renamed `SmartMotorControllerConfig.withZeroOffset` to `SmartMotorControllerConfig.withExternalEncoderZeroOffset`

## 2025.10.31

* Updated `SwerveInputStream` to use `.with*` for shared styling.
* Added `swerve_drive` example.

## 2025.10.29

* Added setpoint visualization to Arm Mechanism2d
* Fixed ArmFeedforward calculation on Spark's, and Nova's. Changed velocity calculation from Rotations to Radians. Found by 8114 (The Rocketeers)

## 2025.10.27

* Renamed `Shooter` to `FlyWheel`
* Added Loop Time to each mechanism.

## 2025.10.14

* Added `YUnits` which allows you to include more exotic units in your code.
* Added `Sensor`s which allow you to seamlessly fetch automatically modified sensor data during sim and real data when running IRL.
