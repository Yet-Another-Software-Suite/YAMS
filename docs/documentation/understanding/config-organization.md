---
icon: folder-tree
---

# Organizing configs in constants

As your robot grows, managing configuration across multiple subsystems can become unwieldy. YAMS supports a clean pattern where you define all your configs in a centralized Constants file, then bind them to subsystems at construction time using `.withSubsystem()`.

## The problem

Without organization, each subsystem file contains its own configuration inline:

```java
// ArmSubsystem.java - config mixed with subsystem logic
public class ArmSubsystem extends SubsystemBase {
  private final Arm arm;
  
  public ArmSubsystem() {
    SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
        .withClosedLoopController(5, 0, 0.1)
        .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
        .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0));
    
    SmartMotorController smc = new TalonFXWrapper(new TalonFX(1, new CANBus(CANPort.CAN_S0)), DCMotor.getKrakenX60(1), smcConfig);
    
    ArmConfig armConfig = new ArmConfig()
        .withLength(Inches.of(18))
        // ... more config
    
    this.arm = new Arm(armConfig, smc);
  }
}
```

This approach has drawbacks:

* Configs are scattered across many files
* Hard to compare settings between mechanisms
* Tuning requires opening multiple files
* CAN IDs, motor types, and gear ratios are buried in subsystem code

## The solution: Centralized constants with `withSubsystem()`

YAMS configs can be created **without** a subsystem reference, then bound later using `.withSubsystem()`. This enables a clean separation:

{% hint style="info" %}
This page uses Commands v2 (`yams.commands2.config.SmartMotorControllerConfig`). With Commands v3, import `yams.commands3.config.SmartMotorControllerConfig` and bind it with `.withMechanism(this)` from your `Mechanism` class instead of `.withSubsystem(this)`. Everything else is the same.
{% endhint %}

```java
// Constants.java - ALL configs in one place
public final class Constants {
  
  public static final class ArmConstants {
    public static final int CAN_ID = 1;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()  // No subsystem yet!
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0))
            .withStartingPosition(Degrees.of(0));  // Where the arm is when the robot turns on
    
    public static final ArmConfig ARM_CONFIG = 
        new ArmConfig()  // No SmartMotorController yet!
            .withLength(Inches.of(18))
            .withHardLimits(Rotations.of(-0.3), Rotations.of(0.3))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  }
  
  public static final class ElevatorConstants {
    public static final int CAN_ID = 2;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withDrumRadius(Inches.of(0.75))
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0))
            .withStartingPosition(Meters.of(0));  // Height when the robot turns on
    
    public static final ElevatorConfig ELEVATOR_CONFIG = 
        new ElevatorConfig()
            .withCarriageWeight(Pounds.of(10))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
  }
  
  public static final class ShooterConstants {
    public static final int CAN_ID = 3;
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01));
    
    public static final FlyWheelConfig FLYWHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
  }
}
```

{% hint style="warning" %}
**Give arms and elevators a starting position.** The starting position lives on the `SmartMotorControllerConfig`, so it belongs in the constant too. Without `withStartingPosition(...)`, creating an `Arm` in simulation throws `Arm starting angle is empty`, and an `Elevator` throws `Starting height is not configured!`. It must also sit inside the mechanism's hard limits.
{% endhint %}

Then in your subsystems, use `.withSubsystem()` to bind the config:

```java
// ArmSubsystem.java - clean and focused
public class ArmSubsystem extends SubsystemBase {
  private final Arm arm;
  
  public ArmSubsystem() {
    // Copy the shared config, then bind the copy to this subsystem
    SmartMotorControllerConfig smcConfig = ArmConstants.SMC_CONFIG.clone()
        .withSubsystem(this);
    
    // Create SmartMotorController with the bound config
    SmartMotorController smc = new TalonFXWrapper(
        new TalonFX(ArmConstants.CAN_ID, new CANBus(CANPort.CAN_S0)), 
        DCMotor.getKrakenX60(1), 
        smcConfig
    );
    
    // Clone ArmConfig and pass smc to the mechanism constructor
    ArmConfig armConfig = ArmConstants.ARM_CONFIG.clone();
    
    this.arm = new Arm(armConfig, smc);
  }
  
  // Commands and business logic only, no config clutter!
  public Command goToAngle(Angle target) {
    return arm.runTo(target, Degrees.of(1));  // tolerance required
  }
}
```

## Benefits of this pattern

| Benefit                    | Description                                            |
| -------------------------- | ------------------------------------------------------ |
| **Single Source of Truth** | All CAN IDs, gear ratios, PID gains in one file        |
| **Easy Comparison**        | See all mechanism configs side-by-side                 |
| **Faster Tuning**          | Change gains without navigating subsystem files        |
| **Cleaner Subsystems**     | Subsystem classes focus on behavior, not configuration |
| **Reusable Configs**       | Share base configs between similar mechanisms          |

## Complete example: Constants file

```java
package first.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.enums.TelemetryVerbosity;

public final class Constants {

  // ==================== ARM ====================
  public static final class ArmConstants {
    public static final int CAN_ID = 1;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4, 3)))
            .withClosedLoopController(5, 0, 0.1)
            .withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
            .withTrapezoidalProfile(RotationsPerSecond.of(1.0), RotationsPerSecondPerSecond.of(2.0))
            .withZeroPower(MotorMode.BRAKE)
            .withStartingPosition(Degrees.of(0)); // Required: Arm throws without it in simulation
    
    public static final ArmConfig ARM_CONFIG = 
        new ArmConfig()
            .withLength(Inches.of(18))
            .withHardLimits(Rotations.of(-0.3), Rotations.of(0.3))
            .withTelemetry("Arm", TelemetryVerbosity.HIGH);
  }

  // ==================== ELEVATOR ====================
  public static final class ElevatorConstants {
    public static final int CAN_ID = 2;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(5, 4)))
            .withDrumRadius(Inches.of(0.75))
            .withClosedLoopController(10, 0, 0.5)
            .withFeedforward(new ElevatorFeedforward(0.1, 0.2, 0.5, 0.01))
            .withTrapezoidalProfile(MetersPerSecond.of(1.0), MetersPerSecondPerSecond.of(2.0))
            .withZeroPower(MotorMode.BRAKE)
            .withStartingPosition(Meters.of(0)); // Required: Elevator throws without it in simulation
    
    public static final ElevatorConfig ELEVATOR_CONFIG = 
        new ElevatorConfig()
            .withCarriageWeight(Pounds.of(10))
            .withHardLimits(Meters.of(0), Meters.of(1.5))
            .withTelemetry("Elevator", TelemetryVerbosity.HIGH);
  }

  // ==================== SHOOTER ====================
  public static final class ShooterConstants {
    public static final int UPPER_CAN_ID = 3;
    public static final int LOWER_CAN_ID = 4;
    public static final DCMotor MOTOR = DCMotor.getKrakenX60(1);
    
    public static final SmartMotorControllerConfig SMC_CONFIG = 
        new SmartMotorControllerConfig()
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(1)))
            .withClosedLoopController(0.5, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0.1, 0.12, 0.01))
            .withZeroPower(MotorMode.COAST);
    
    public static final FlyWheelConfig FLYWHEEL_CONFIG = 
        new FlyWheelConfig()
            .withDiameter(Inches.of(4))
            .withTelemetry("Shooter", TelemetryVerbosity.HIGH);
  }
}
```

## Complete example: Subsystem using constants

```java
package first.robot.subsystems;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import first.robot.Constants.ArmConstants;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

public class ArmSubsystem extends SubsystemBase {

  private final Arm arm;

  public ArmSubsystem() {
    // Step 1: Copy the shared SmartMotorControllerConfig and bind the copy to this subsystem
    SmartMotorControllerConfig smcConfig = ArmConstants.SMC_CONFIG.clone()
        .withSubsystem(this);

    // Step 2: Create SmartMotorController
    SmartMotorController smc = new TalonFXWrapper(
        new TalonFX(ArmConstants.CAN_ID, new CANBus(CANPort.CAN_S0)),
        ArmConstants.MOTOR,
        smcConfig
    );

    // Step 3: Clone ArmConfig and pass smc to the mechanism constructor
    ArmConfig armConfig = ArmConstants.ARM_CONFIG.clone();

    // Step 4: Create Arm mechanism
    this.arm = new Arm(armConfig, smc);
  }

  // ==================== COMMANDS ====================
  
  public Command runToAngle(Angle target) {
    return arm.runTo(target, Degrees.of(1));  // tolerance required
  }

  public Command holdAngle(Angle target) {
    return arm.run(target);
  }

  public Command stow() {
    return arm.runTo(Rotations.of(0), Degrees.of(1));  // tolerance required
  }

  // ==================== ACCESSORS ====================
  
  public Arm getArm() {
    return arm;
  }
}
```

## After live tuning: copy the values into Constants

[Live tuning](../how-to-tuning/live-tuning/) changes gains while the robot runs, but the values are lost when the code restarts. With this pattern the gains live in `Constants`, so that is where you copy them:

```java
// Before
.withClosedLoopController(5, 0, 0.1)
.withFeedforward(new ArmFeedforward(0.1, 0.3, 0.5, 0.01))
// After: the kP, kI, kD and kS, kG, kV, kA you found
.withClosedLoopController(8, 0, 0.3)
.withFeedforward(new ArmFeedforward(0.12, 0.45, 0.5, 0.01))
```

If the config also has `withSimClosedLoopController(...)` or `withSimFeedforward(...)`, values found in simulation belong in those lines, because the simulation uses them.

## Organizing multiple config files

For larger robots, you may want to split configs into separate files:

```
first/robot/
├── Constants.java           // General constants (CAN IDs, ports)
├── configs/
│   ├── ArmConfigs.java      // Arm-related configs
│   ├── ElevatorConfigs.java // Elevator-related configs
│   ├── ShooterConfigs.java  // Shooter-related configs
│   └── DriveConfigs.java    // Swerve/drive configs
└── subsystems/
    ├── ArmSubsystem.java
    ├── ElevatorSubsystem.java
    └── ...
```

{% hint style="info" %}
**Important**: When using this pattern, you **must** call `.withSubsystem()` (`.withMechanism()` in Commands v3) before passing the `SmartMotorControllerConfig` to a `SmartMotorController`. It can only be called once per config and throws a `SmartMotorControllerConfigurationException` if a subsystem is already set, so call `.clone()` on the shared constant first and bind the copy. Never pass the same config instance to more than one `SmartMotorController`. `clone()` copies the settings but shares the gain objects (the PID controllers and feedforwards) with the original, so do not clone one constant for two motors that run their own closed loop. Give each mechanism its own constant. Pass the `SmartMotorController` directly as the second argument to the mechanism constructor (e.g., `new Arm(armConfig, smc)`).
{% endhint %}

## When not to use this pattern

This pattern adds a layer of indirection. For simple robots with 1-2 mechanisms, inline configuration may be clearer. Consider using centralized configs when:

* You have 3+ mechanisms
* Multiple team members tune different subsystems
* You want to quickly compare settings across mechanisms
* You're building a constants dashboard or config loader
