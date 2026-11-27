# SwerveDrive

**Package (robot code):** `yams.commands2.swerve` (Commands v2) or `yams.commands3.swerve` (Commands v3)\
**Extends:** `yams.core.mechanisms.swerve.SwerveDrive`\
**Javadoc:** [`SwerveDrive`](https://javadocs.yassrobotics.com/yams/yams/commands2/swerve/SwerveDrive.html) (Commands v2), [`SwerveDrive`](https://javadocs.yassrobotics.com/yams/yams/commands3/swerve/SwerveDrive.html) (Commands v3)

Manages a complete swerve drivetrain: kinematics, module states, odometry via `SwerveDrivePoseEstimator`, and optional vision measurement fusion.

The state, odometry and simulation logic lives in `yams.core.mechanisms.swerve.SwerveDrive`, which robot code does not construct directly. Import the `SwerveDrive` from the package matching your command framework; it adds the `Subsystem` (v2) or `Mechanism` (v3) binding and the `Command` factories.

Java swerve APIs use the WPILib 2027 kinematics types: `ChassisVelocities` (formerly `ChassisSpeeds`) and `SwerveModuleVelocity` (formerly `SwerveModuleState`), both in `org.wpilib.math.kinematics`.

## Constructor

```java
SwerveDrive(SwerveDriveConfig config)
```

| Parameter | Type                | Description                                                                                                                                                                                             |
| --------- | ------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `config`  | `SwerveDriveConfig` | Full drivetrain configuration including modules, kinematics, and gyro. Use `yams.commands2.config.SwerveDriveConfig` with the v2 drive and `yams.commands3.config.SwerveDriveConfig` with the v3 drive. |

On a real robot (not in simulation) the constructor throws `IllegalStateException` if no gyro was set with `SwerveDriveConfig.withGyro(...)`.

***

## Commands

The command factories differ between the two frameworks.

{% tabs %}
{% tab title="Commands v2" %}
| Method                                                          | Returns     | Description                                                                                                                    |
| --------------------------------------------------------------- | ----------- | ------------------------------------------------------------------------------------------------------------------------------ |
| `drive(Supplier<ChassisVelocities> robotRelativeChassisSpeeds)` | `Command`   | Continuously sets robot-relative chassis velocities from a supplier. Requires the drive's `Subsystem`. Runs until interrupted. |
| `driveToPose(Pose2d pose)`                                      | `Command`   | Resets the translation/rotation PID controllers, then continuously drives toward the field-relative `pose` until interrupted.  |
| `getSubsystem()`                                                | `Subsystem` | The `Subsystem` set on the `SwerveDriveConfig`.                                                                                |
{% endtab %}

{% tab title="Commands v3" %}
| Method                                                                             | Returns     | Description                                                                                                                 |
| ---------------------------------------------------------------------------------- | ----------- | --------------------------------------------------------------------------------------------------------------------------- |
| `drive(Supplier<ChassisVelocities> robotRelativeChassisSpeeds)`                    | `Command`   | Continuously sets robot-relative chassis velocities from a supplier. Requires the drive's `Mechanism`. Runs until canceled. |
| `driveToPose(Pose2d pose)`                                                         | `Command`   | Resets the PID controllers, then drives toward the field-relative `pose` until canceled. Stops the drive when canceled.     |
| `driveToPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance)` | `Command`   | Same, but ends once the robot is within both tolerances, then stops the drive.                                              |
| `isNear(Pose2d pose, Distance translationTolerance, Angle rotationTolerance)`      | `boolean`   | Whether the robot is within both tolerances of `pose`.                                                                      |
| `stop()`                                                                           | `void`      | Commands zero chassis velocities.                                                                                           |
| `getMechanism()`                                                                   | `Mechanism` | The `Mechanism` set on the `SwerveDriveConfig`.                                                                             |
{% endtab %}
{% endtabs %}

***

## Driving

| Method                                                                               | Returns                  | Description                                                                                                                                                                                                                                                                                                                                                                                                                   |
| ------------------------------------------------------------------------------------ | ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `setRobotRelativeChassisSpeeds(ChassisVelocities speeds)`                            | `void`                   | Directly sets robot-relative velocities. Call from `periodic()` or a command.                                                                                                                                                                                                                                                                                                                                                 |
| `setRobotRelativeChassisSpeeds(ChassisVelocities speeds, Force[] feedforwardForces)` | `void`                   | Same, plus a per-module drive-wheel feedforward `Force`, FL/FR/BL/BR order (matching module order). Pass an empty array for no feedforward; any other length that does not match the module count throws `IllegalArgumentException`. Designed to plug PathPlanner's `AutoBuilder.configure(...)` `driveRobotRelative` consumer straight in: `(speeds, ff) -> drive.setRobotRelativeChassisSpeeds(speeds, ff.linearForces())`. |
| `setFieldRelativeChassisSpeeds(ChassisVelocities speeds)`                            | `void`                   | Sets field-relative velocities, rotating them by `getRotation()`, the robot's rotation from odometry, before applying.                                                                                                                                                                                                                                                                                                        |
| `setSwerveModuleStates(SwerveModuleVelocity[] states)`                               | `void`                   | Directly commands module states, bypassing chassis-velocities-to-states kinematics.                                                                                                                                                                                                                                                                                                                                           |
| `setSwerveModuleStates(SwerveModuleVelocity[] states, Force[] feedforwardForces)`    | `void`                   | Same, plus a per-module drive-wheel feedforward `Force` (FL/FR/BL/BR order).                                                                                                                                                                                                                                                                                                                                                  |
| `getStateFromRobotRelativeChassisSpeeds(ChassisVelocities speeds)`                   | `SwerveModuleVelocity[]` | Converts robot-relative velocities to module states, desaturated by `withMaximumModuleSpeed(...)` (and `withMaximumChassisSpeed(...)` if set).                                                                                                                                                                                                                                                                                |
| `lockPose()`                                                                         | `void`                   | Turns every module toward the robot center without driving it (the X pattern), making the robot hard to push. Uses `SwerveModule.setAzimuthAngle(Rotation2d)`, which skips the velocity deadband, so the wheels actually turn. Call it every loop while locking, e.g. from a command.                                                                                                                                         |
| `setAzimuthAngles(Rotation2d angle)`                                                 | `void`                   | Turns every module to `angle` without driving it, as for tuning the azimuth motors. Uses `SwerveModule.setAzimuthAngle(Rotation2d)`, so the velocity deadband does not hold the wheels where they are, and records the desired states for telemetry. Used by swerve azimuth live tuning.                                                                                                                                      |

***

## Gyro

| Method                  | Returns      | Description                                                                                                                                                                                                                                                                                           |
| ----------------------- | ------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getGyroRotation3d()`   | `Rotation3d` | The robot's attitude from the gyro (see [`SwerveDriveConfig.getGyroRotation3d()`](swerve-drive-config.md#gyro)). Its yaw (`getZ()`, or `toRotation2d()`) is the robot heading, wrapped to +/-180 degrees; its roll and pitch are the robot's tilt. In simulation, the simulated heading with no tilt. |
| `getRotation()`         | `Rotation2d` | The robot's rotation from odometry, the heading of `getPose()`, which vision measurements correct. Field relative driving, `getFieldRelativeSpeed()`, driving to a pose and `SwerveInputStream`'s heading control use it to turn field relative speeds into module states.                            |
| `zeroOdometryHeading()` | `void`       | Resets the odometry to the same position facing 0 (toward the red alliance), so `getRotation()` and field-relative "forward" read 0. The gyro is not changed.                                                                                                                                         |
| `zeroGyro()`            | `void`       | Same as `zeroOdometryHeading()`.                                                                                                                                                                                                                                                                      |

{% hint style="warning" %}
`getGyroAngle()` was removed from `SwerveDrive` and `SwerveDriveConfig`. Use `getGyroRotation3d()` instead, e.g. `drive.getGyroRotation3d().toRotation2d()` for the heading as a `Rotation2d`.
{% endhint %}

***

## Auto-align (drive to pose)

| Method                                                                           | Returns             | Description                                                                                                                                                                                                                                                                                                          |
| -------------------------------------------------------------------------------- | ------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `driveToPoseSetpoint(Pose2d targetPose)`                                         | `ChassisVelocities` | Computes one loop's robot-relative `ChassisVelocities` toward `targetPose` using the configured PID controllers. Lower-level building block behind `driveToPose(...)`; reset the PID controllers yourself before looping on this. Throws `SwerveDriveConfigurationException` if either controller is not configured. |
| `driveToPoseSetpoint(Pose2d targetPose, Pose2d currentPose, Rotation2d heading)` | `ChassisVelocities` | Same, from a given pose and heading (e.g. replayed from a log).                                                                                                                                                                                                                                                      |
| `getDistanceFromPose(Pose2d pose)`                                               | `Distance`          | Distance from `pose` to the robot.                                                                                                                                                                                                                                                                                   |
| `getAngleDifferenceFromPose(Pose2d pose)`                                        | `Angle`             | Heading difference between the robot and `pose`.                                                                                                                                                                                                                                                                     |
| `setTranslationPID(PIDController controller)`                                    | `void`              | Replaces the translation controller used by `driveToPose(...)`. Only resets the controller (clearing its integrator) if P, I, or D actually changed, so live tuning doesn't wipe accumulated state every loop.                                                                                                       |
| `setRotationPID(PIDController controller)`                                       | `void`              | Replaces the rotation controller used by `driveToPose(...)`, with the same change-detection guard as `setTranslationPID`.                                                                                                                                                                                            |
| `resetTranslationPID()`                                                          | `void`              | Resets the translation controller's internal state (e.g. integrator).                                                                                                                                                                                                                                                |
| `resetRotationPID()`                                                             | `void`              | Resets the rotation controller's internal state.                                                                                                                                                                                                                                                                     |

{% hint style="info" %}
When both a translation and a rotation controller are configured, `SwerveDrive` publishes a live-tuning command at `Mechanisms/<name>/tuning` (a Commands v2 `Command` in v2, or a Commands v3 `Command` wrapped in a `CommandTunable` in v3; see [Telemetry & DataLog](swerve-drive.md#telemetry--datalog) below). Running it resets both PID controllers, then every loop checks a tunable `AutoAlignEnabled` boolean and, while `true`, drives toward a tunable target pose (`autoalign/setpoint/x`/`autoalign/setpoint/y`/`autoalign/setpoint/rot`). This is useful for tuning `withTranslationController`/`withRotationController` gains, or driving to an arbitrary pose, without redeploying code. The `TranslationP/I/D`/`RotationP/I/D` tuning entries are pre-seeded from the PID controllers passed to `withTranslationController`/`withRotationController` (not `0`), the same way `SmartMotorControllerTelemetry` seeds its `kP`/`kI`/`kD` entries from the motor's configured PID.
{% endhint %}

***

## Odometry and pose

| Method                                                                        | Returns                  | Description                                                                                                                                                                                           |
| ----------------------------------------------------------------------------- | ------------------------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `getPose()`                                                                   | `Pose2d`                 | Returns the current estimated robot pose.                                                                                                                                                             |
| `resetOdometry(Pose2d pose)`                                                  | `void`                   | Resets the pose estimator to `pose`. The gyro is not changed: the pose estimator keeps the offset between the gyro and the pose, and field-relative driving uses `getRotation()`, the pose's heading. |
| `addVisionMeasurement(Pose2d pose, double timestamp)`                         | `void`                   | Fuses a vision-derived pose into the estimator at `timestamp` (FPGA seconds).                                                                                                                         |
| `addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> stdDevs)` | `void`                   | Same, with per-measurement standard deviations.                                                                                                                                                       |
| `setVisionMeasurementStdDevs(Matrix<N3, N1> stdDevs)`                         | `void`                   | Sets the estimator's default trust in vision measurements (x, y in meters, theta in radians).                                                                                                         |
| `getRobotRelativeSpeed()`                                                     | `ChassisVelocities`      | Measured robot-relative velocities, from the module states.                                                                                                                                           |
| `getFieldRelativeSpeed()`                                                     | `ChassisVelocities`      | Measured field-relative velocities.                                                                                                                                                                   |
| `getModuleStates()`                                                           | `SwerveModuleVelocity[]` | Returns the current state (speed and angle) of each module.                                                                                                                                           |
| `getModulePositions()`                                                        | `SwerveModulePosition[]` | Returns the integrated position of each module.                                                                                                                                                       |
| `getDesiredChassisSpeeds()`                                                   | `ChassisVelocities`      | Returns the last-commanded robot-relative setpoint (not a measurement).                                                                                                                               |
| `getDesiredModuleStates()`                                                    | `SwerveModuleVelocity[]` | Returns the last-commanded module states (not a measurement).                                                                                                                                         |
| `getRobotRelativeChassisSpeedsFromState(SwerveModuleVelocity[])`              | `ChassisVelocities`      | Converts an arbitrary set of module states back into robot-relative chassis velocities.                                                                                                               |
| `getKinematics()`                                                             | `SwerveDriveKinematics`  | Builds a `SwerveDriveKinematics` from the module locations.                                                                                                                                           |
| `getSimPose()`                                                                | `Pose2d`                 | Returns the simulated ground-truth pose. See below.                                                                                                                                                   |
| `getField2d()`                                                                | `Field2d`                | Returns the `Field2d` the drive already publishes (at `Mechanisms/<name>/field`). Add your own `FieldObject2d`s to it instead of creating another `Field2d`.                                          |
| `getModule(String moduleName)`                                                | `Optional<SwerveModule>` | The module with the given name, if any.                                                                                                                                                               |
| `getModules()`                                                                | `SwerveModule[]`         | All modules of the drive.                                                                                                                                                                             |
| `getConfig()`                                                                 | `SwerveDriveConfig<?>`   | The drive's configuration.                                                                                                                                                                            |
| `getName()`                                                                   | `String`                 | The telemetry name of the drive.                                                                                                                                                                      |

{% hint style="warning" %}
`getDesiredChassisSpeeds()` returns a **setpoint**, not the robot's actual measured motion: it is whatever was last passed to `setRobotRelativeChassisSpeeds(...)` (directly, or via `setFieldRelativeChassisSpeeds(...)`/`drive(...)`), cached and republished every `updateTelemetry()` call. If nothing has commanded the drive yet, it returns a zeroed `ChassisVelocities`. For the drive's actual measured speed, use `getRobotRelativeSpeed()` / `getFieldRelativeSpeed()` instead.
{% endhint %}

{% hint style="info" %}
There is no direct accessor for the underlying `SwerveDrivePoseEstimator`. `SwerveDrive` owns it exclusively to avoid callers accidentally calling `update(...)`/`resetPosition(...)`/`resetPose(...)` on it directly (which would corrupt the pose estimate by feeding it duplicate or out-of-order samples, or desyncing it from the drive's gyro offset). Use `getPose()`, `resetOdometry(Pose2d)`, and `addVisionMeasurement(...)` instead; they cover the estimator interactions `SwerveDrive` supports.
{% endhint %}

{% hint style="info" %}
`getSimPose()` returns a separate, ground-truth `Pose2d` that assumes every module reached its last-commanded `SwerveModuleVelocity` perfectly; it is **not** the same as `getPose()` (the noisy, gyro/odometry-fused estimate). It's updated in simulation by `simIterate()`, by integrating a `Twist2d` built from the desired module states, and is also snapped to the given pose by `resetOdometry(Pose2d)` so it stays in sync with the fused estimate. On a real robot it stays at the configured starting pose. This makes it a convenient "known truth" pose to feed into a simulated vision system (e.g. to generate synthetic AprilTag detections) so you can test vision code end-to-end without a physical camera.
{% endhint %}

***

## Lifecycle

| Method              | Returns | Description                                                                                                        |
| ------------------- | ------- | ------------------------------------------------------------------------------------------------------------------ |
| `updateTelemetry()` | `void`  | Updates the pose estimator and publishes pose, module states, and speeds to NetworkTables. Call from `periodic()`. |
| `simIterate()`      | `void`  | Advances the swerve physics simulation by one loop iteration. Call from `simulationPeriodic()`.                    |

***

## Telemetry and DataLog

`updateTelemetry()` publishes the pose, the robot's rotation from odometry (`rotation`, a `Rotation2d`), the gyro's rotation (`gyro`, a `Rotation2d`), chassis speeds and module states to NetworkTables under `Mechanisms/<name>` at the verbosity configured via `SwerveDriveConfig.withTelemetry(String name, TelemetryVerbosity)` (default `HIGH`). Tunable values are published under `Tuning/<name>`. It also calls `SwerveModule.updateTelemetry()` for each module in the drive.

For full control over exactly which fields are published (including the auto-align PID gains `TranslationP/I/D`, `RotationP/I/D`, the tunable target pose fields `AutoAlignPoseX`/`AutoAlignPoseY`/`AutoAlignPoseRotation` (`autoalign/setpoint/x`, `autoalign/setpoint/y`, `autoalign/setpoint/rot`), the `AutoAlignEnabled` boolean, and the shared module tuning fields `ModulesDriveP/I/D`/`ModulesDriveKs/Kv/Ka`/`ModulesDriveVelocity`/`ModulesDriveTuningEnabled`/`ModulesDriveInPlace`/`ModulesDriveVoltage`/`ModulesDriveVoltageEnabled` (`modules/drive/feedback/p,i,d`, `modules/drive/feedforward/s,v,a`, `modules/drive/velocity`, `modules/drive/enabled`, `modules/drive/inplace`, `modules/drive/voltage/setpoint`, `modules/drive/voltage/enabled`) and `ModulesAzimuthP/I/D`/`ModulesAzimuthKs/Kv/Ka`/`ModulesAzimuthAngle`/`ModulesAzimuthTuningEnabled`/`ModulesAzimuthVoltage`/`ModulesAzimuthVoltageEnabled` (`modules/azimuth/feedback/p,i,d`, `modules/azimuth/feedforward/s,v,a`, `modules/azimuth/angle`, `modules/azimuth/enabled`, `modules/azimuth/voltage/setpoint`, `modules/azimuth/voltage/enabled`), all used by the live-tuning dashboard command), pass a `SwerveDriveTelemetryConfig` (`yams.core.telemetry`) to `SwerveDriveConfig.withTelemetry(String name, SwerveDriveTelemetryConfig)`. The module tuning fields apply to every module on the drive at once, not to a single module, and the feedback/feedforward gains are pre-seeded from the first module's configured PID controller and `SimpleMotorFeedforward`, not `0`. To additionally record fields to a WPILib DataLog (for offline review in AdvantageScope), call `.withDataLogName(String)` on that `SwerveDriveTelemetryConfig`; see [SwerveDriveConfig](swerve-drive-config.md#datalog-telemetry).

{% hint style="warning" %}
`AutoAlignEnabled`, `ModulesDriveTuningEnabled`, and `ModulesAzimuthTuningEnabled` are mutually exclusive. applyTuningValues() reads which tuning modes are on, and a mode that is on turns the ones it outranks back off (priority: auto-align, then drive tuning, then azimuth tuning), forcing their NetworkTables value back to `false`. Auto-align drives to its target pose every loop. When the modes change, the new modes' setpoints are sent: drive tuning commands every module through `SwerveDrive.setSwerveModuleStates(...)`, at `0°` or, with `ModulesDriveInPlace`, tangent to its position around the robot center (its location angle plus `90°`) so a positive `ModulesDriveVelocity` spins the robot counter-clockwise; azimuth tuning turns every module to the tuned angle with `0` drive velocity; and with no mode on, every wheel is held where it is. After that, only values that changed are applied. `ModulesDriveVoltageEnabled` and `ModulesAzimuthVoltageEnabled`, while their tuning mode is on, drive the drive or azimuth motors at `ModulesDriveVoltage` or `ModulesAzimuthVoltage` volts instead, without their closed loops, as for characterizing them: the drive wheels point as for drive tuning and are not optimized, and the drive motors are held at the coupling velocity while the azimuth turns. A closed loop that runs on the SystemCore is stopped while its voltage is on and started again when it turns off. When the tuning command ends, `SwerveDriveTelemetry.stopTuning(drive)` turns every mode off and applies it once more, so a voltage stops and the wheels are held.
{% endhint %}

***

## Example usage

{% tabs %}
{% tab title="Commands v2" %}
```java
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.kinematics.ChassisVelocities;
import java.util.function.Supplier;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;

public class SwerveSubsystem extends SubsystemBase {
    private final SwerveDrive m_drive;

    public SwerveSubsystem() {
        m_drive = new SwerveDrive(new SwerveDriveConfig(this, fl, fr, bl, br)
            .withGyro(m_gyro::getRotation3d));
    }

    public Command driveCommand(Supplier<ChassisVelocities> robotRelativeSpeeds) {
        return m_drive.drive(robotRelativeSpeeds);
    }

    public Command lock() {
        return run(m_drive::lockPose);
    }

    @Override
    public void periodic() {
        m_drive.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        m_drive.simIterate();
    }
}
```
{% endtab %}

{% tab title="Commands v3" %}
```java
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.kinematics.ChassisVelocities;
import java.util.function.Supplier;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;

public class SwerveMechanism implements Mechanism {
    private final SwerveDrive m_drive;

    public SwerveMechanism() {
        m_drive = new SwerveDrive(new SwerveDriveConfig(this, fl, fr, bl, br)
            .withGyro(m_gyro::getRotation3d));
    }

    public Command driveCommand(Supplier<ChassisVelocities> robotRelativeSpeeds) {
        return m_drive.drive(robotRelativeSpeeds);
    }

    public Command lock() {
        return run(coroutine -> {
            while (true) {
                m_drive.lockPose();
                coroutine.yield();
            }
        }).named("Swerve Lock");
    }

    // Called from the robot's periodic loop.
    public void periodic() {
        m_drive.updateTelemetry();
    }

    public void simulationPeriodic() {
        m_drive.simIterate();
    }
}
```
{% endtab %}
{% endtabs %}

Full examples: [Commands v2 `SwerveSubsystem`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/SwerveSubsystem.java) and [Commands v3 `SwerveMechanism`](https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/SwerveMechanism.java).

***

## See also

* [SwerveDriveConfig](swerve-drive-config.md)
* [SwerveModule](swerve-module.md)
* [SwerveInputStream](swerve-input-stream.md)
* [C++ SwerveDrive](../../c++-reference/swerve/swerve-drive.md)
