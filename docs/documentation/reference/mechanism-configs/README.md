---
icon: sitemap
---

# Mechanism configs

Every mechanism is built from two configs: a mechanism config that says what the mechanism is, and a [`SmartMotorControllerConfig`](../editor/config/) that says how its motor behaves.

<figure><img src="../../.gitbook/assets/mech-two-configs.png" alt="A diagram. A mechanism config, which says what the mechanism is, such as its length, mass, hard stops and how it is drawn, and a SmartMotorControllerConfig, which says how its motor behaves, such as gearing, PID, feedforward, soft limits and current, both go into the mechanism class. The mechanism then drives the simulation&#x27;s physics and Mechanism2d, and the real robot&#x27;s commands and telemetry."><figcaption><p>Two configs go into one mechanism.</p></figcaption></figure>

```java
SmartMotorController motor  = new SparkWrapper(spark, DCMotor.getNEO(1), motorConfig); // how the motor behaves
ArmConfig            config = new ArmConfig()                                          // what the arm is
    .withLength(Feet.of(3))
    .withHardLimits(Degrees.of(-30), Degrees.of(110))
    .withTelemetry("Arm", TelemetryVerbosity.HIGH);
Arm arm = new Arm(config, motor);
```

## The pages

| Page                                          | Config               | For                                                       |
| --------------------------------------------- | -------------------- | --------------------------------------------------------- |
| [Shared options](shared-options.md)           | all four below       | Telemetry, hard limits, and how the mechanism is drawn.   |
| [ArmConfig](arm-config.md)                    | `ArmConfig`          | Arms that swing against gravity.                          |
| [ElevatorConfig](elevator-config.md)          | `ElevatorConfig`     | Elevators and linear slides.                              |
| [PivotConfig](pivot-config.md)                | `PivotConfig`        | Turrets and wrists. A pivot does not account for gravity. |
| [FlyWheelConfig](flywheel-config.md)          | `FlyWheelConfig`     | Shooters, rollers, intakes and anything else that spins.  |
| [SwerveDriveConfig](swerve-drive-config.md)   | `SwerveDriveConfig`  | The whole swerve drive base.                              |
| [SwerveModuleConfig](swerve-module-config.md) | `SwerveModuleConfig` | One swerve module.                                        |

## Which options each config has

| Option                                                                                                             | Arm      | Elevator    | Pivot    | FlyWheel |
| ------------------------------------------------------------------------------------------------------------------ | -------- | ----------- | -------- | -------- |
| [`withTelemetry`](shared-options.md#withtelemetry)                                                                 | ✅        | ✅           | ✅        | ✅        |
| [`withSimColor`](shared-options.md#withsimcolor)                                                                   | ✅        | ✅           | ✅        | ✅        |
| [`withMechanismPositionConfig`](shared-options.md#withmechanismpositionconfig)                                     | ✅        | ✅           | ✅        | ✅        |
| [`withHardLimits`](shared-options.md#withhardlimits)                                                               | ✅ angles | ✅ distances | ✅ angles | ❌        |
| [`withLength`](arm-config.md#withlength)                                                                           | ✅        | ❌           | ❌        | ❌        |
| [`withCarriageWeight`](elevator-config.md#withcarriageweight)                                                      | ❌        | ✅           | ❌        | ❌        |
| [`withAngle`](elevator-config.md#withangle), [`withHorizontalElevator`](elevator-config.md#withhorizontalelevator) | ❌        | ✅           | ❌        | ❌        |
| [`withDiameter`](flywheel-config.md#withdiameter)                                                                  | ❌        | ❌           | ❌        | ✅        |
| [`withSpeedometerSimulation`](flywheel-config.md#withspeedometersimulation)                                        | ❌        | ❌           | ❌        | ✅        |

The swerve configs are different enough to have their own pages and their own option lists.

## What simulation needs

{% hint style="warning" %}
Most mechanism config options describe the mechanism to the simulator and do nothing on the real robot. Some of them are **required** in simulation: if one is missing, the mechanism throws an exception when it is created in simulation, and the robot code stops. A missing option on the mechanism config throws that mechanism's own exception, such as an `ArmConfigurationException` or a `PivotConfigurationException`. A missing option on the motor config, such as an elevator's starting height, throws a `SmartMotorControllerConfigurationException`. Each option page says where the option is used.
{% endhint %}

| Mechanism  | Must be set for simulation to start                                                                                                                                                                                                                                                                                                                             |
| ---------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `Arm`      | [`withLength`](arm-config.md#withlength), [`withHardLimits`](shared-options.md#withhardlimits), and a [starting position](../editor/config/encoders.md#withstartingposition) inside the hard limits.                                                                                                                                                            |
| `Elevator` | [`withCarriageWeight`](elevator-config.md#withcarriageweight), [`withHardLimits`](shared-options.md#withhardlimits) or [`withStages`](elevator-config.md#withstages), a [circumference](../editor/config/gearing-and-units.md#withmechanismcircumference), and a [starting position](../editor/config/encoders.md#withstartingposition) inside the hard limits. |
| `Pivot`    | [`withHardLimits`](shared-options.md#withhardlimits), and a [starting position](../editor/config/encoders.md#withstartingposition) inside the hard limits.                                                                                                                                                                                                      |
| `FlyWheel` | Nothing on the mechanism config. Its moment of inertia comes from the [motor config](../editor/config/simulation.md#withmomentofinertia).                                                                                                                                                                                                                       |

## Options that live on the motor config

Several things you might look for on a mechanism config are on the motor's config, because they change what the motor controller does:

| Looking for                                            | It is                                                                                                                                                                  |
| ------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Soft limits                                            | [`withSoftLimits`](../editor/config/limits-and-protection.md#withsoftlimits)                                                                                           |
| Where the mechanism starts                             | [`withStartingPosition`](../editor/config/encoders.md#withstartingposition)                                                                                            |
| Gear ratio, drum or wheel size                         | [Gearing and units](../editor/config/gearing-and-units.md)                                                                                                             |
| Gains, feedforward, motion profile                     | [Closed loop control](../editor/config/closed-loop-control.md), [Feedforward](../editor/config/feedforward.md), [Motion profiles](../editor/config/motion-profiles.md) |
| Mass or moment of inertia of an arm, pivot or flywheel | [`withMomentOfInertia`](../editor/config/simulation.md#withmomentofinertia)                                                                                            |
| An absolute encoder                                    | [Encoders](../editor/config/encoders.md)                                                                                                                               |

## Code reference

* [`ArmConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/ArmConfig.java)
* [`ElevatorConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/ElevatorConfig.java)
* [`PivotConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/PivotConfig.java)
* [`FlyWheelConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/FlyWheelConfig.java)
* [`SwerveDriveConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/SwerveDriveConfig.java)
* [`SwerveModuleConfig`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/yams/java/yams/core/mechanisms/config/SwerveModuleConfig.java)
