---
icon: car-side
---

# Swerve drive with AdvantageKit

YAMS swerve integrates seamlessly with AdvantageKit. Here's a complete example based on the official YAMS AdvantageKit swerve examples ([Commands v2 `SwerveSubsystem`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/advantage_kit/java/first/robot/subsystems/SwerveSubsystem.java), [Commands v3 `SwerveMechanism`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/advantage_kit/java/first/robot/mechanisms/SwerveMechanism.java)):

{% hint style="info" %}
WPILib 2027 renamed the swerve kinematics types: `ChassisSpeeds` is now `ChassisVelocities` and `SwerveModuleState` is now `SwerveModuleVelocity`. The YAMS swerve API uses the new names.
{% endhint %}

```java
package first.robot.subsystems;

import static org.wpilib.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {

  private AngularVelocity maxAngularVelocity = DegreesPerSecond.of(720);
  private LinearVelocity maxLinearVelocity = MetersPerSecond.of(4);
  private Supplier<Angle> gyroAngleSupplier;
  private SwerveDriveConfig config;

  /** AdvantageKit inputs for the swerve drive */
  @AutoLog
  public static class SwerveInputs {
    public SwerveModulePosition[] positions = new SwerveModulePosition[4];
    public SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    public Angle gyroRotation = Degrees.of(0);
    public ChassisVelocities robotRelativeSpeeds = new ChassisVelocities(0, 0, 0);
    public Pose2d estimatedPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  }

  private final SwerveInputsAutoLogged swerveInputs = new SwerveInputsAutoLogged();
  private final SwerveDrive drive;

  public SwerveModule createModule(SparkMax driveMotor, SparkMax azimuthMotor,
      CANcoder absoluteEncoder, String moduleName, Translation2d location) {

    MechanismGearing driveGearing = new MechanismGearing(GearBox.fromStages("12:1", "2:1"));
    MechanismGearing azimuthGearing = new MechanismGearing(GearBox.fromStages("21:1"));

    SmartMotorControllerConfig driveCfg = new SmartMotorControllerConfig(this)
        .withWheelDiameter(Inches.of(4))
        .withClosedLoopController(50, 0, 4)
        .withGearing(driveGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

    SmartMotorControllerConfig azimuthCfg = new SmartMotorControllerConfig(this)
        .withClosedLoopController(50, 0, 4)
        .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
        .withGearing(azimuthGearing)
        .withStatorCurrentLimit(Amps.of(20))
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

    SmartMotorController driveSMC = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveCfg);
    SmartMotorController azimuthSMC = new SparkWrapper(azimuthMotor, DCMotor.getNEO(1), azimuthCfg);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveSMC, azimuthSMC)
        .withAbsoluteEncoder(absoluteEncoder.getAbsolutePosition().asSupplier())
        .withTelemetry(moduleName, TelemetryVerbosity.HIGH)
        .withLocation(location)
        .withOptimization(true);

    return new SwerveModule(moduleConfig);
  }

  public SwerveInputStream getChassisSpeedsSupplier(DoubleSupplier x, DoubleSupplier y,
      DoubleSupplier rot) {
    return SwerveInputStream.of(drive, x, y, rot)
        .withMaximumAngularVelocity(maxAngularVelocity)
        .withMaximumLinearVelocity(maxLinearVelocity)
        .withDeadband(0.01)
        .withCubeRotationControllerAxis()
        .withCubeTranslationControllerAxis()
        .withAllianceRelativeControl();
  }

  public SwerveSubsystem() {
    Pigeon2 gyro = new Pigeon2(14, new CANBus(CANPort.CAN_S0));
    gyroAngleSupplier = gyro.getYaw().asSupplier();

    var fl = createModule(new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless),
        new SparkMax(CANPorts.fromBusId(1), 2, MotorType.kBrushless),
        new CANcoder(3, new CANBus(CANPort.CAN_S0)),
        "frontleft", new Translation2d(Inches.of(12), Inches.of(12)));
    var fr = createModule(new SparkMax(CANPorts.fromBusId(1), 4, MotorType.kBrushless),
        new SparkMax(CANPorts.fromBusId(1), 5, MotorType.kBrushless),
        new CANcoder(6, new CANBus(CANPort.CAN_S0)),
        "frontright", new Translation2d(Inches.of(12), Inches.of(-12)));
    var bl = createModule(new SparkMax(CANPorts.fromBusId(1), 7, MotorType.kBrushless),
        new SparkMax(CANPorts.fromBusId(1), 8, MotorType.kBrushless),
        new CANcoder(9, new CANBus(CANPort.CAN_S0)),
        "backleft", new Translation2d(Inches.of(-12), Inches.of(12)));
    var br = createModule(new SparkMax(CANPorts.fromBusId(1), 10, MotorType.kBrushless),
        new SparkMax(CANPorts.fromBusId(1), 11, MotorType.kBrushless),
        new CANcoder(12, new CANBus(CANPort.CAN_S0)),
        "backright", new Translation2d(Inches.of(-12), Inches.of(-12)));

    config = new SwerveDriveConfig(this, fl, fr, bl, br)
        // The gyro supplier returns a Rotation3d built from the LOGGED yaw, so replay is deterministic
        .withGyro(() -> new Rotation3d(getGyroAngle()))
        .withStartingPose(swerveInputs.estimatedPose)
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0));

    drive = new SwerveDrive(config);
  }

  private Rotation2d getGyroAngle() {
    return new Rotation2d(swerveInputs.gyroRotation);
  }

  private void updateInputs() {
    swerveInputs.estimatedPose = drive.getPose();
    swerveInputs.states = drive.getModuleStates();
    swerveInputs.positions = drive.getModulePositions();
    swerveInputs.robotRelativeSpeeds = drive.getRobotRelativeSpeed();
    swerveInputs.gyroRotation = gyroAngleSupplier.get();
  }

  /** Drive with field relative speeds (e.g. from a SwerveInputStream) using the logged pose's heading. */
  public Command setFieldRelativeChassisSpeeds(Supplier<ChassisVelocities> fieldRelativeSpeeds) {
    return run(() -> setRobotRelativeChassisSpeedsSetpoint(
        fieldRelativeSpeeds.get().toRobotRelative(getPose().getRotation())))
        .withName("Set Field Relative Chassis Speeds Supplier");
  }

  private void setRobotRelativeChassisSpeedsSetpoint(ChassisVelocities speeds) {
    Logger.recordOutput("Swerve/DesiredChassisSpeeds", speeds);
    SwerveModuleVelocity[] states = drive.getStateFromRobotRelativeChassisSpeeds(speeds);
    Logger.recordOutput("Swerve/DesiredStates", states);
    drive.setSwerveModuleStates(states);
  }

  /** PID drive to a pose using the logged pose and gyro angle, so replay follows the same path. */
  public Command driveToPose(Pose2d pose) {
    return startRun(() -> {
      drive.resetTranslationPID();
      drive.resetRotationPID();
    }, () -> drive.setRobotRelativeChassisSpeeds(
        drive.driveToPoseSetpoint(pose, getPose(), getPose().getRotation())))
        .withName("Drive to Pose");
  }

  /** Point every module at its corner (X pattern) without driving it, logging the desired states. */
  public Command lock() {
    return run(() -> {
      SwerveModule[] modules = config.getModules();
      SwerveModuleVelocity[] desiredStates = new SwerveModuleVelocity[modules.length];
      for (int i = 0; i < modules.length; i++) {
        // Each module points at its own corner, which makes the X.
        // setAzimuthAngle turns the wheel without driving it and returns the state it set.
        // A zero speed state through setSwerveModuleStates would be held where the wheel is
        // by the velocity deadband.
        desiredStates[i] = modules[i].setAzimuthAngle(
            modules[i].getConfig().getLocation().orElseThrow().getAngle().orElse(Rotation2d.ZERO));
      }
      Logger.recordOutput("Swerve/DesiredStates", desiredStates);
    }).withName("Lock");
  }

  public Pose2d getPose() { return swerveInputs.estimatedPose; }
  public ChassisVelocities getRobotRelativeSpeeds() { return swerveInputs.robotRelativeSpeeds; }

  @Override
  public void periodic() {
    drive.updateTelemetry();  // Updates pose estimator, call BEFORE updateInputs
    updateInputs();
    Logger.processInputs("Swerve", swerveInputs);
  }

  @Override
  public void simulationPeriodic() {
    drive.simIterate();
  }
}
```

The `SwerveInputStream` output is field relative, so bind it through `setFieldRelativeChassisSpeeds`. `withTelemetry(...)` on the stream publishes its mode and configuration, and at `HIGH` adds a `Live Tuning` command (see [SwerveInputStream](../../reference/swerve-input-stream/)):

```java
// In RobotContainer
drive.setDefaultCommand(drive.setFieldRelativeChassisSpeeds(
    drive.getChassisSpeedsSupplier(() -> -xboxController.getLeftY(),
                                   () -> -xboxController.getLeftX(),
                                   () -> -xboxController.getRightX())
         .withTelemetry("Driver", TelemetryVerbosity.HIGH)));
```

{% hint style="warning" %}
`SwerveDrive.lockPose()` turns every module to its corner (the X pattern) with zero drive speed, but it is marked as not compatible with AdvantageKit. The YAMS AdvantageKit example builds the same X pattern itself in `lock()`, turning each module with `SwerveModule.setAzimuthAngle(...)` so the [velocity deadband](../../understanding/swerve-drive/swerve-module.md#velocity-deadband) doesn't hold the wheels where they are, and the desired states are recorded with `Logger.recordOutput()` like every other drive command.
{% endhint %}

**Check:** run the code in simulation and drive with the left stick. In AdvantageScope, `AdvantageKit/Swerve/EstimatedPose` on a 2D field moves with the robot, and `AdvantageKit/RealOutputs/Swerve/DesiredStates` changes as you drive.

## Commands v3

The [Commands v3 AdvantageKit example](../../../../examples/commands3/advantage_kit) uses the same inputs class and module setup, with these differences:

* The class implements `org.wpilib.command3.Mechanism` instead of extending `SubsystemBase`, and imports `yams.commands3.config.SmartMotorControllerConfig`, `yams.commands3.config.SwerveDriveConfig`, `yams.commands3.swerve.SwerveDrive` and `yams.commands3.swerve.SwerveInputStream`.
* Commands are coroutines (`while (true)` + `coroutine.yield()`).
* `periodic()` and `simulationPeriodic()` are plain methods that `Robot.robotPeriodic()` and `Robot.simulationPeriodic()` call before `scheduler.run()`.
* The teleop opmode creates the `SwerveInputStream` and hands it to the drive mechanism.

```java
// SwerveMechanism
private SwerveInputStream inputStream;

public Command driveInputStream() {
  return run(coroutine -> {
    while (true) {
      setFieldRelativeChassisSpeeds(inputStream.get());
      coroutine.yield();
    }
  }).named("Swerve Drive Input Stream");
}

public void setInputStream(SwerveInputStream inputStream) {
  // Stop publishing the old stream's telemetry before replacing it
  if (this.inputStream != null) {
    this.inputStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
  }
  this.inputStream = inputStream;
}

public SwerveDrive getSwerveDrive() {
  return drive;
}
```

```java
// AngularVelocityTeleop, created from Robot.teleopInit()
robot.drive.setInputStream(SwerveInputStream.of(
        robot.drive.getSwerveDrive(),
        () -> -controller.getLeftY(),
        () -> -controller.getLeftX(),
        () -> -controller.getRightX())
    .withDeadband(0.01)
    .withCubeRotationControllerAxis()
    .withCubeTranslationControllerAxis()
    .withAllianceRelativeControl()
    .withTelemetry("Driver", TelemetryVerbosity.HIGH));
robot.drive.setDefaultCommand(robot.drive.driveInputStream());
```

## Key points for swerve with AdvantageKit

1. **Create an `@AutoLog` inputs class** with module states, positions, gyro, and pose
2. **Call `updateTelemetry()` BEFORE `updateInputs()`**: the pose estimator and internal state update in `updateTelemetry()`
3. **Feed the gyro from the logged yaw** (`withGyro(() -> new Rotation3d(getGyroAngle()))`) so odometry is deterministic in replay
4. **Turn field relative speeds into robot relative ones with the logged pose's heading** (`getPose().getRotation()`), the odometry heading YAMS drives with, so replay gives the same module states
5. **Log desired states and speeds** using `Logger.recordOutput()` for replay debugging
6. **Call `simIterate()` in `simulationPeriodic()`** for physics simulation

{% hint style="warning" %}
**Important: Replay Data Scope**

During AdvantageKit log replay, **only data inside the `@AutoLog` inputs classes is mutated**. YAMS's built-in telemetry (published via `withTelemetry()`) is NOT affected by replay: it will show live simulation values, not replayed values.

This means:

* **Inputs you define** (in `SwerveInputs`, `ArmIOInputs`, etc.): replayed correctly
* **YAMS telemetry** (from `.withTelemetry("Arm")`): shows live sim values during replay

To ensure accurate replay debugging, always pull the data you need into your `@AutoLog` inputs class rather than relying on YAMS telemetry during replay.
{% endhint %}
