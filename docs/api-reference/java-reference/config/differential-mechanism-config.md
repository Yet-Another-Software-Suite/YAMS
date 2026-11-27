# DifferentialMechanismConfig

**Package:** `yams.core.mechanisms.config`\
**Javadoc:** [`DifferentialMechanismConfig`](https://javadocs.yassrobotics.com/yams/yams/core/mechanisms/config/DifferentialMechanismConfig.html)

`DifferentialMechanismConfig` configures a two-motor differential mechanism. Unlike the other config classes, it holds the two motor controllers. Supply them in the constructor, or with `withSmartMotorControllers(...)` once; they cannot be replaced after they are set.

All builder methods return the `DifferentialMechanismConfig` instance for chaining.

See also: [DifferentialMechanism](../mechanisms/differential-mechanism.md)

***

## Constructor

```java
DifferentialMechanismConfig(SmartMotorController left, SmartMotorController right)
DifferentialMechanismConfig()  // call withSmartMotorControllers(...) before use
DifferentialMechanismConfig clone()  // Copy
```

| Parameter | Type                   | Description                                                                        |
| --------- | ---------------------- | ---------------------------------------------------------------------------------- |
| `left`    | `SmartMotorController` | The "left" motor. Its output mixes with the right motor to produce tilt and twist. |
| `right`   | `SmartMotorController` | The "right" motor.                                                                 |

Both motor controllers, a starting tilt and twist, and a length are **required** before the config is used by a `DifferentialMechanism`; it throws `DifferentialMechanismConfigurationException` otherwise. The default movement plane is XY.

***

## Builder methods

| Method                                                                             | Parameters                                             | Description                                                                                                                                               |
| ---------------------------------------------------------------------------------- | ------------------------------------------------------ | --------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `withSmartMotorControllers(SmartMotorController left, SmartMotorController right)` | `left`, `right`: motor controllers                     | Sets the motor controllers when the no-argument constructor was used. Throws `DifferentialMechanismConfigurationException` if they are already set.       |
| `withStartingPosition(Angle tilt, Angle twist)`                                    | `tilt`, `twist`: starting angles                       | **Required** (or set both individually below). Seeds both encoders with an initial tilt and twist.                                                        |
| `withTiltStartingPosition(Angle startingPosition)`                                 | `startingPosition`: starting tilt angle                | Seeds the tilt encoder with an initial position. A tilt starting position is required.                                                                    |
| `withTwistStartingPosition(Angle startingPosition)`                                | `startingPosition`: starting twist angle               | Seeds the twist encoder with an initial position. A twist starting position is required.                                                                  |
| `withAngleSuppliers(Supplier<Angle> twistSupplier, Supplier<Angle> tiltSupplier)`  | absolute angle suppliers                               | Supplies absolute encoder readings for twist and tilt. Optional.                                                                                          |
| `withGearing(MechanismGearing gearing)`                                            | `gearing`: bevel gearing                               | Twist gearing when the bevel gears are not 1:1, separate from the motor gearing. Optional.                                                                |
| `withLength(Distance length)`                                                      | `length`: arm length                                   | **Required.** Sets the mechanism arm length, used to compute the end position (also set by `withMOI(Distance, Mass)`).                                    |
| `withMOI(Distance length, Mass weight)`                                            | `length`: arm length, `weight`: arm mass               | Sets the length and estimates the moment of inertia for simulation.                                                                                       |
| `withMOI(MomentOfInertia MOI)`                                                     | `MOI`: moment of inertia                               | Sets the moment of inertia for simulation directly. An MOI is required in simulation.                                                                     |
| `withTelemetry(String name, TelemetryVerbosity verbosity)`                         | `name`: NT key prefix, `verbosity`: `LOW`/`MID`/`HIGH` | Enables NetworkTables telemetry for this mechanism. Optional.                                                                                             |
| `withSimColor(Color8Bit color)`                                                    | `color`: RGB color                                     | Sets the Mechanism2d visualization color. Optional.                                                                                                       |
| `withMechanismPositionConfig(MechanismPositionConfig config)`                      | `config`: position config                              | Where the mechanism is on the robot, for `getPose3d()`, and the plane it moves in. See [MechanismPositionConfig](mechanism-position-config.md). Optional. |

Other methods: `applyConfig()` reapplies both motor controller configs, and the getters `getLeftMotorController()`, `getRightMotorController()`, `getMOI()`, `getLength()`, `getTwistGearing()`, `getStartingTiltAngle()`, `getStartingTwistAngle()`, `getTiltAngleSupplier()`, `getTwistAngleSupplier()`, `getTelemetryName()`, `getTelemetryVerbosity()`, `getSimColor()` and `getMechanismPositionConfig()`. `getTiltAngle(Angle left, Angle right)`, `getTwistAngle(Angle left, Angle right)`, `getLeftMechanismPosition(Angle tilt, Angle twist)` and `getRightMechanismPosition(Angle tilt, Angle twist)` convert between motor and tilt/twist positions.

***

## Example

```java
SmartMotorController leftSMC = new SparkWrapper(
    new SparkMax(CANPort.CAN_S0, 1, MotorType.kBrushless),
    DCMotor.getNEO(1),
    leftConfig
);
SmartMotorController rightSMC = new SparkWrapper(
    new SparkMax(CANPort.CAN_S0, 2, MotorType.kBrushless),
    DCMotor.getNEO(1),
    rightConfig
);

DifferentialMechanismConfig config = new DifferentialMechanismConfig(leftSMC, rightSMC)
    .withStartingPosition(Degrees.of(90), Degrees.of(0))
    .withMOI(Inches.of(12), Pounds.of(4))
    .withTelemetry("Wrist", TelemetryVerbosity.HIGH);
```

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/DiffyMechSubsystem.java#L64-L68" %}
