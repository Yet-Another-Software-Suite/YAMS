# Setting up swerve drive

**Goal:** Create a fully functional `SwerveDrive` subsystem with field-relative driving, odometry, and simulation.

The gains and encoder offsets in this guide are placeholders, as [the overview](guides.md#the-gains-in-these-guides) explains. The [Swerve Drive tutorial](https://yams.yassrobotics.com/documentation/tutorials/swerve-drive) explains each setting, how to tune it, and how to measure the offsets on the robot.

***

{% stepper %}
{% step %}
#### Configure drive and azimuth motor controllers for each module

Each swerve module needs two `SmartMotorController` instances: one for drive (velocity) and one for azimuth (position). Follow [Configuring a Motor Controller](configuring-a-motor.md) for each.

Drive controllers use velocity PID and a `SimpleMotorFeedforward`. Azimuth controllers use position PID with continuous wrapping, and optionally a `SimpleMotorFeedforward` for friction compensation. The complete class in [Construct `SwerveDrive` and wire command factories](setting-up-swerve.md#construct-swervedrive-and-wire-command-factories) builds both in a `createModule(...)` helper.

**Check it:** the project builds with the vendordep for your motor controllers and CANcoders installed.
{% endstep %}

{% step %}
#### Create a `SwerveModuleConfig` for each module

Pass the drive and azimuth motor controllers into the `SwerveModuleConfig` constructor. Then set the wheel radius, the module's location relative to the robot center, and the absolute encoder.

```java
SwerveModuleConfig frontLeftConfig = new SwerveModuleConfig(driveMotorFL, azimuthMotorFL)
    .withWheelRadius(Meters.of(0.0508))
    .withLocation(new Translation2d(0.381, 0.381))
    .withAbsoluteEncoder(cancoderFL.getAbsolutePosition().asSupplier())
    .withAbsoluteEncoderOffset(Rotations.of(0.24))
    .withTelemetry("FrontLeft", TelemetryVerbosity.HIGH);
```

Repeat for `FrontRight`, `BackLeft`, and `BackRight`, adjusting the `Translation2d` signs and encoder offsets. `withLocation` takes meters from the robot center, with +x toward the front and +y toward the left.

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Build `SwerveModule` instances and create a `SwerveDriveConfig`

Wrap each `SwerveModuleConfig` in a `SwerveModule`, then collect the four modules into a `SwerveDriveConfig`. Pass the subsystem (Commands v2) or mechanism (Commands v3) and all four modules in the constructor, then set kinematic limits and the gyro supplier.

`SwerveModuleConfig` and `SwerveModule` come from `yams.core`. `SwerveDriveConfig`, `SwerveDrive`, and `SwerveInputStream` come from `yams.commands2` or `yams.commands3`.

```java
SwerveModule frontLeft  = new SwerveModule(frontLeftConfig);
SwerveModule frontRight = new SwerveModule(frontRightConfig);
SwerveModule backLeft   = new SwerveModule(backLeftConfig);
SwerveModule backRight  = new SwerveModule(backRightConfig);

SwerveDriveConfig driveConfig = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
    .withGyro(gyro::getRotation3d)
    .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(360))
    .withTranslationController(new PIDController(1.0, 0, 0))
    .withRotationController(new PIDController(1.0, 0, 0))
    .withTelemetry("swerve", TelemetryVerbosity.HIGH);
```

`withGyro` takes a `Supplier<Rotation3d>` for the robot's attitude, such as `pigeon::getRotation3d`. Its yaw is the robot's heading, and its roll and pitch are what anti-tipping (`withAntiTipping`) corrects with.

{% hint style="info" %}
**Gyro mounted on its side?** Call `.withGyroHeadingAxis(GyroAxis.ROLL)` or `.withGyroHeadingAxis(GyroAxis.PITCH)` (`yams.core.mechanisms.config.enums.GyroAxis`) to tell YAMS which gyro axis points up through the robot. The default is `GyroAxis.YAW`, for a gyro mounted flat.
{% endhint %}

**Check it:** the project builds.
{% endstep %}

{% step %}
#### Construct `SwerveDrive` and wire command factories

{% tabs %}
{% tab title="Commands v2" %}
```java
package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class DriveSubsystem extends SubsystemBase {

    private final CANBus canBus = new CANBus(CANPort.CAN_S0);
    private final Pigeon2 gyro = new Pigeon2(14, canBus);
    private final SwerveDrive swerveDrive;

    public DriveSubsystem() {
        // Placeholder encoder offsets: measure each module's offset on the robot.
        SwerveModule frontLeft  = createModule("FrontLeft",  10, 11, 12, new Translation2d(0.381, 0.381),   Rotations.of(0.24));
        SwerveModule frontRight = createModule("FrontRight", 20, 21, 22, new Translation2d(0.381, -0.381),  Rotations.of(0.0));
        SwerveModule backLeft   = createModule("BackLeft",   30, 31, 32, new Translation2d(-0.381, 0.381),  Rotations.of(0.0));
        SwerveModule backRight  = createModule("BackRight",  40, 41, 42, new Translation2d(-0.381, -0.381), Rotations.of(0.0));

        SwerveDriveConfig driveConfig = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
            .withGyro(gyro::getRotation3d)
            .withMaximumChassisSpeed(MetersPerSecond.of(4.5), DegreesPerSecond.of(360))
            .withTranslationController(new PIDController(1.0, 0, 0)) // Placeholder auto-align gains.
            .withRotationController(new PIDController(1.0, 0, 0))
            .withTelemetry("swerve", TelemetryVerbosity.HIGH);

        swerveDrive = new SwerveDrive(driveConfig);
    }

    /** Build one module: its drive and azimuth motors, its CANcoder, and where it sits on the robot. */
    private SwerveModule createModule(String name, int driveId, int azimuthId, int encoderId,
                                      Translation2d location, Angle encoderOffset) {
        SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
            .withZeroPower(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
            .withClosedLoopController(0.1, 0.0, 0.0)                         // Placeholder PID gains.
            .withFeedforward(new SimpleMotorFeedforward(0.12, 2.2, 0.3))    // Placeholder kS, kV, kA.
            .withStatorCurrentLimit(Amps.of(60));

        SmartMotorControllerConfig azimuthConfig = new SmartMotorControllerConfig(this)
            .withZeroPower(MotorMode.BRAKE)
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(12.8)))
            .withClosedLoopController(7.0, 0.0, 0.3)                         // Placeholder PID gains.
            .withContinuousWrapping(Radians.of(-Math.PI), Radians.of(Math.PI))
            .withStatorCurrentLimit(Amps.of(40));

        SmartMotorController driveMotor = new TalonFXWrapper(
            new TalonFX(driveId, canBus), DCMotor.getKrakenX60(1), driveConfig);
        SmartMotorController azimuthMotor = new TalonFXWrapper(
            new TalonFX(azimuthId, canBus), DCMotor.getKrakenX60(1), azimuthConfig);
        CANcoder cancoder = new CANcoder(encoderId, canBus);

        SwerveModuleConfig moduleConfig = new SwerveModuleConfig(driveMotor, azimuthMotor)
            .withWheelRadius(Meters.of(0.0508))
            .withLocation(location)
            .withAbsoluteEncoder(cancoder.getAbsolutePosition().asSupplier())
            .withAbsoluteEncoderOffset(encoderOffset)
            .withTelemetry(name, TelemetryVerbosity.HIGH);

        return new SwerveModule(moduleConfig);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /** Field-relative drive command driven by a SwerveInputStream. */
    public Command driveFieldRelativeCommand(SwerveInputStream inputStream) {
        return run(() -> swerveDrive.setFieldRelativeChassisSpeeds(inputStream.get()))
            .withName("Field Oriented Drive");
    }

    /** Turn the wheels into an X pattern to resist being pushed. */
    public Command lock() {
        return run(swerveDrive::lockPose).withName("Lock");
    }

    /** Pass vision pose estimates to the odometry estimator. */
    public void addVisionMeasurement(Pose2d pose, double timestampSeconds) {
        swerveDrive.addVisionMeasurement(pose, timestampSeconds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /** Robot attitude from the gyro. Its yaw is the robot's heading. */
    public Rotation3d getGyroRotation3d() {
        return swerveDrive.getGyroRotation3d();
    }

    @Override
    public void periodic() {
        swerveDrive.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        swerveDrive.simIterate();
    }
}
```
{% endtab %}

{% tab title="Commands v3" %}
The hardware and config code is identical. The class implements `Mechanism`, the YAMS command-layer imports come from `yams.commands3`, and the command factories are coroutines.

```java
package first.robot.mechanisms;

import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
// ... the yams.core, org.wpilib.math, and vendor imports are the same as Commands v2

public class DriveMechanism implements Mechanism {

    private final SwerveDrive swerveDrive;

    public DriveMechanism() {
        // ... same motor, module, and SwerveDriveConfig setup as Commands v2
        swerveDrive = new SwerveDrive(driveConfig);
    }

    public SwerveDrive getSwerveDrive() {
        return swerveDrive;
    }

    /** Field-relative drive command driven by a SwerveInputStream. Runs until canceled. */
    public Command driveFieldRelativeCommand(SwerveInputStream inputStream) {
        return run(coroutine -> {
            while (true) {
                swerveDrive.setFieldRelativeChassisSpeeds(inputStream.get());
                coroutine.yield();
            }
        }).named("Field Oriented Drive");
    }

    /** Turn the wheels into an X pattern to resist being pushed. Runs until canceled. */
    public Command lock() {
        return run(coroutine -> {
            while (true) {
                swerveDrive.lockPose();
                coroutine.yield();
            }
        }).named("Lock");
    }

    /** Call from Robot.robotPeriodic(), before the scheduler runs. */
    public void periodic() {
        swerveDrive.updateTelemetry();
    }

    /** Run in its own Notifier, started from Robot.simulationInit(). */
    public void simulationPeriodic() {
        swerveDrive.simIterate();
    }
}
```

In `Robot`, call `drive.periodic()` from `robotPeriodic()` before `scheduler.run()`, and start the simulation in `simulationInit()` with `startSimulation("Drive", drive::simulationPeriodic)`, as in [Building an arm subsystem](building-an-arm.md#integrate-into-a-subsystem).
{% endtab %}
{% endtabs %}

{% hint style="info" %}
`SwerveDrive` already publishes a `Field2d` with the robot pose. Use `swerveDrive.getField2d()` if you want to draw on it instead of creating your own.
{% endhint %}

**Check it:** run the simulation. The code starts without an exception, and the drive shows in NetworkTables under **Mechanisms -> swerve**, with a field showing the robot at the origin.
{% endstep %}

{% step %}
#### Set up driver input with `SwerveInputStream`

`SwerveInputStream` translates raw joystick axes into field-relative `ChassisVelocities` (WPILib 2027's name for `ChassisSpeeds`), applying deadbands and scaling. Pass the `SwerveDrive` instance as the first argument to `SwerveInputStream.of()`. Rotation is set with `.withControllerRotationAxis()`, or as a fourth argument to `of()`.

```java
SwerveInputStream driverInput = SwerveInputStream.of(
    driveSubsystem.getSwerveDrive(),      // SwerveDrive instance
    () -> -driverController.getLeftY(),   // forward/back
    () -> -driverController.getLeftX()    // strafe
).withControllerRotationAxis(() -> -driverController.getRightX())
 .withDeadband(0.1)
 .withMaximumLinearVelocity(MetersPerSecond.of(4.5))
 .withMaximumAngularVelocity(DegreesPerSecond.of(360))
 .withAllianceRelativeControl()
 .withTelemetry("Driver", TelemetryVerbosity.HIGH);

driveSubsystem.setDefaultCommand(
    driveSubsystem.driveFieldRelativeCommand(driverInput)
);
```

`withTelemetry(name, verbosity)` publishes the stream every time it is read; at `HIGH` you can also edit it live. Call it last, after the rest of the configuration. [SwerveInputStream](../java-reference/swerve/swerve-input-stream.md) lists what each verbosity publishes.

**Check it:** in simulation, enable teleop with a controller connected (or a keyboard joystick in the Sim GUI). The left stick drives the robot on the field and the right stick turns it.
{% endstep %}

{% step %}
#### Azimuth encoder seeding

`seedAzimuthEncoder()` is called automatically on each `SwerveModule` during construction. No additional call is required at startup. If you need to re-seed after the CAN bus has fully settled, you can call `seedAzimuthEncoder()` on each `SwerveModule` instance individually.

**Check it:** on the robot, with the robot disabled, turn each wheel by hand. Its azimuth angle in telemetry follows the wheel. If a wheel points the wrong way when enabled, its encoder offset is wrong.
{% endstep %}

{% step %}
#### Auto-align (`driveToPose`) and live PID tuning

`driveToPose(Pose2d)` returns a `Command` that drives the robot to a field-relative pose using the `withTranslationController`/`withRotationController` PID gains set on `SwerveDriveConfig`:

```java
public Command driveToPose(Pose2d target) {
    return swerveDrive.driveToPose(target);
}
```

At `TelemetryVerbosity.HIGH`, `SwerveDrive` also publishes a live tuning command for these gains and for the module gains, so you can tune them without redeploying. [SwerveDrive: Auto-Align](../java-reference/swerve/swerve-drive.md#auto-align-drive-to-pose) lists its fields.

**Check it:** in simulation, schedule `driveToPose(new Pose2d(2, 1, Rotation2d.kZero))` from a button. The robot drives to that point on the field and stops.
{% endstep %}
{% endstepper %}

***

## Notes

{% hint style="info" %}
For the robot's heading, use `swerveDrive.getGyroRotation3d().toRotation2d()`. The heading wraps at +/-180 degrees.
{% endhint %}

{% hint style="info" %}
`lockPose()` turns each wheel toward the robot center (an X pattern) without driving it. Call it every loop from a command, as in `lock()` above.
{% endhint %}

{% hint style="info" %}
Simulated SPARK MAX and SPARK FLEX controllers close their loop at 1 kHz, like the real hardware. See [Simulation](simulation.md).
{% endhint %}

## If it did not work

* **The robot drives the wrong way, or spins when it should drive straight.** Check each module's `withLocation` signs, then that the drive motors are not inverted.
* **A wheel points the wrong way, or the wheels fight each other.** The absolute encoder offsets are wrong. Measure them as in [On the real robot](https://yams.yassrobotics.com/documentation/tutorials/swerve-drive/real-robot).
* **Field-relative driving is rotated.** The gyro supplier or `withGyroHeadingAxis` is wrong, or the gyro was not zeroed facing away from the driver.
* **Nothing moves in simulation.** Commands v2: check that `simulationPeriodic()` calls `simIterate()`. Commands v3: check that `simulationInit()` calls `startSimulation(...)`.

[Troubleshooting](https://yams.yassrobotics.com/documentation/tutorials/troubleshooting) covers more.

***

## Examples

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands2/simple_robot-2027/src/main/java/first/robot/subsystems/SwerveSubsystem.java" %}

Commands v3 version of the swerve drive:

{% @github-files/github-code-block url="https://github.com/Yet-Another-Software-Suite/YAMS/blob/master/examples/commands3/simple_robot-2027/src/main/java/first/robot/mechanisms/SwerveMechanism.java" %}

***

## Related pages

* [SwerveDrive](../java-reference/swerve/swerve-drive.md)
* [SwerveInputStream](../java-reference/swerve/swerve-input-stream.md)
* [SwerveDriveConfig](../java-reference/swerve/swerve-drive-config.md)
* [Configuring a Motor Controller](configuring-a-motor.md)
* [Simulation](simulation.md)
