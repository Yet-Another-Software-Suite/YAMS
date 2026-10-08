// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.mechanisms;

import static first.robot.Constants.SwerveConstants.*;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.DriveConstants;
import first.robot.Field;
import first.robot.Ports;
import first.robot.Vision;
import java.util.List;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.hardware.rotation.AnalogEncoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
import yams.commands3.telemetry.SwerveInputStreamTelemetry;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Swerve drivetrain built with YAMS: four NEO SPARK MAX modules with Thrifty absolute encoders, the
 * Systemcore IMU as the gyro, and two Limelights for vision.
 *
 * <p>The original was YAGSL configured from JSON. The same gearing, offsets, inversions, gains and
 * current limits are set here in code.
 */
public class Swerve implements Mechanism {
    /** Tolerances at a waypoint the robot drives through. */
    private static final Distance kWaypointTolerance = Meters.of(0.3);
    private static final Angle kWaypointHeadingTolerance = Degrees.of(15);
    /** Tolerances at a pose the robot stops at. */
    private static final Distance kEndTolerance = Meters.of(0.05);
    private static final Angle kEndHeadingTolerance = Degrees.of(3);

    private final OnboardIMU gyro = new OnboardIMU(MountOrientation.FLAT);
    private final SwerveDrive drive;
    /** Active driver input, set by the teleop opmode. */
    private SwerveInputStream inputStream;
    private final List<Vision> cameras = List.of(Vision.drivetrainCamera(), Vision.turretCamera());
    private final DoublePublisher distanceToHubPublisher = NetworkTableInstance.getDefault()
        .getDoubleTopic("Swerve/Distance to Hub (m)")
        .publish();

    public Swerve() {
        final SwerveModule frontLeft = createModule("frontleft", Ports.kFrontLeftDrive, Ports.kFrontLeftSteer,
            Ports.kFrontLeftEncoder, kFrontLeftEncoderOffset,
            new Translation2d(DriveConstants.kModuleOffset, DriveConstants.kModuleOffset));
        final SwerveModule frontRight = createModule("frontright", Ports.kFrontRightDrive, Ports.kFrontRightSteer,
            Ports.kFrontRightEncoder, kFrontRightEncoderOffset,
            new Translation2d(DriveConstants.kModuleOffset, DriveConstants.kModuleOffset.unaryMinus()));
        final SwerveModule backLeft = createModule("backleft", Ports.kBackLeftDrive, Ports.kBackLeftSteer,
            Ports.kBackLeftEncoder, kBackLeftEncoderOffset,
            new Translation2d(DriveConstants.kModuleOffset.unaryMinus(), DriveConstants.kModuleOffset));
        final SwerveModule backRight = createModule("backright", Ports.kBackRightDrive, Ports.kBackRightSteer,
            Ports.kBackRightEncoder, kBackRightEncoderOffset,
            new Translation2d(DriveConstants.kModuleOffset.unaryMinus(), DriveConstants.kModuleOffset.unaryMinus()));

        final SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
            .withGyro(gyro::getRotation3d)
            // The original started the robot on the red side of the field, facing the blue wall.
            .withStartingPose(new Pose2d(13, 4, Rotation2d.k180deg))
            .withMaximumChassisSpeed(DriveConstants.kMaxSpeed, DriveConstants.kMaxAngularSpeed)
            .withMaximumModuleSpeed(DriveConstants.kMaxSpeed)
            // Heading PID from the YAGSL controller properties, used to aim at the hub.
            .withRotationController(new PIDController(kHeadingKP, 0, kHeadingKD))
            // Drive to pose, used by the autos, with the original PathPlanner translation gain.
            .withTranslationController(new PIDController(kTranslationKP, 0, 0))
            .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
        drive = new SwerveDrive(config);
    }

    private SwerveModule createModule(String name, int driveId, int steerId, int encoderChannel, Angle encoderOffset,
                                      Translation2d location) {
        final SparkMax driveMotor = new SparkMax(Ports.kCANPort, driveId, MotorType.kBrushless);
        final SparkMax steerMotor = new SparkMax(Ports.kCANPort, steerId, MotorType.kBrushless);
        // Thrifty absolute encoder on an analog input; reads the module angle in rotations.
        final AnalogEncoder encoder = new AnalogEncoder(encoderChannel);

        final SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kDriveGearRatio))
            .withWheelDiameter(kWheelDiameter)
            .withClosedLoopController(kDriveKP, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, kDriveKV))
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(kDriveCurrentLimit)
            .withClosedLoopRampRate(kRampRate)
            .withOpenLoopRampRate(kRampRate)
            .withMotorInverted(kDriveInverted)
            .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

        final SmartMotorControllerConfig steerConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kSteerGearRatio))
            .withClosedLoopController(kSteerKP, 0, 0)
            .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(kSteerCurrentLimit)
            .withClosedLoopRampRate(kRampRate)
            .withOpenLoopRampRate(kRampRate)
            .withMotorInverted(kSteerInverted)
            .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

        final SmartMotorController driveController = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig);
        final SmartMotorController steerController = new SparkWrapper(steerMotor, DCMotor.getNEO(1), steerConfig);

        return new SwerveModule(new SwerveModuleConfig(driveController, steerController)
            .withLocation(location)
            .withOptimization(true)
            // The steering SPARK's encoder is seeded from the Thrifty encoder at startup.
            .withAbsoluteEncoder(encoder::get)
            .withAbsoluteEncoderOffset(encoderOffset)
            .withTelemetry(name, TelemetryVerbosity.HIGH));
    }

    /**
     * Drive with field relative {@link ChassisVelocities}, e.g. from a {@link SwerveInputStream}
     * built in an opmode. Runs until canceled.
     *
     * @param velocities Field relative {@link ChassisVelocities}, read every loop.
     * @return {@link Command} that drives the robot.
     */
    public Command driveFieldRelative(Supplier<ChassisVelocities> velocities) {
        return run(coroutine -> {
            while (true) {
                drive.setFieldRelativeChassisSpeeds(velocities.get());
                coroutine.yield();
            }
        }).named("Swerve Drive Field Relative");
    }

    /**
     * Drive with the active {@link SwerveInputStream}. A stream set while this runs takes effect on the next loop. Runs
     * until canceled.
     *
     * @return {@link Command} that drives the robot.
     */
    public Command driveInputStream() {
        return run(coroutine -> {
            while (true) {
                drive.setFieldRelativeChassisSpeeds(inputStream.get());
                coroutine.yield();
            }
        }).named("Swerve Drive Input Stream");
    }

    /**
     * Drive slowly with a copy of the active {@link SwerveInputStream} until canceled.
     *
     * @return {@link Command} that drives the robot.
     */
    public Command slowMode() {
        return driveInputStreamCopy(stream -> stream.withScaleTranslation(DriveConstants.kSlowModeTranslationScale),
            "Slow Mode");
    }

    /**
     * Drive slowly with a copy of the active {@link SwerveInputStream} while facing this alliance's hub, until
     * canceled.
     *
     * @return {@link Command} that drives the robot.
     */
    public Command aimAtHub() {
        return driveInputStreamCopy(stream -> stream.withScaleTranslation(DriveConstants.kAimTranslationScale)
            .withAim(() -> new Pose2d(Field.hub(), Rotation2d.ZERO), () -> true), "Aim at Hub");
    }

    /**
     * Drive with a modified copy of the active {@link SwerveInputStream}, leaving the active stream unchanged.
     *
     * @param modify Changes to make to the copy.
     * @param name   Command name.
     * @return {@link Command} that drives the robot.
     */
    private Command driveInputStreamCopy(UnaryOperator<SwerveInputStream> modify, String name) {
        return run(coroutine -> {
            SwerveInputStream stream = modify.apply(inputStream.clone());
            while (true) {
                drive.setFieldRelativeChassisSpeeds(stream.get());
                coroutine.yield();
            }
        }).named(name);
    }

    /**
     * Set the active driver input.
     *
     * @param inputStream Field relative {@link SwerveInputStream} driven by {@link #driveInputStream()}.
     */
    public void setInputStream(SwerveInputStream inputStream) {
        // Stop publishing the replaced stream's telemetry, e.g. when another teleop opmode is selected.
        if (this.inputStream != null) {
            this.inputStream.getTelemetry().ifPresent(SwerveInputStreamTelemetry::close);
        }
        this.inputStream = inputStream;
    }

    /**
     * Get the active driver input, so commands can read or adjust it.
     *
     * @return Active {@link SwerveInputStream}.
     */
    public SwerveInputStream getInputStream() {
        return inputStream;
    }

    /**
     * Hold position and face this alliance's hub, until canceled. For autonomous, where no driver input is set.
     *
     * @return {@link Command} that aims the robot.
     */
    public Command aimAtHubInPlace() {
        return run(coroutine -> {
            SwerveInputStream stream = SwerveInputStream.of(drive, () -> 0, () -> 0, () -> 0)
                .withAim(() -> new Pose2d(Field.hub(), Rotation2d.ZERO), () -> true);
            while (true) {
                drive.setFieldRelativeChassisSpeeds(stream.get());
                coroutine.yield();
            }
        }).named("Aim at Hub in Place");
    }

    /**
     * Drive through a waypoint: drive toward a blue pose, flipped for the red alliance, ending once the robot is near
     * it so it carries on into the next waypoint without stopping.
     *
     * @param bluePose Field relative {@link Pose2d}, blue alliance origin.
     * @return {@link Command} that ends near the pose.
     */
    public Command driveThroughPose(Pose2d bluePose) {
        return driveToPose(Field.forAlliance(bluePose), kWaypointTolerance, kWaypointHeadingTolerance);
    }

    /**
     * Drive to a blue pose, flipped for the red alliance, and stop there.
     *
     * @param bluePose Field relative {@link Pose2d}, blue alliance origin.
     * @return {@link Command} that ends at the pose.
     */
    public Command driveToPose(Pose2d bluePose) {
        return driveToPose(Field.forAlliance(bluePose), kEndTolerance, kEndHeadingTolerance);
    }

    /**
     * Drive to a pose, ending once the robot is within the given tolerances of it.
     *
     * @param pose                 Field relative {@link Pose2d}, blue alliance origin.
     * @param translationTolerance Maximum distance from the pose to be considered there.
     * @param rotationTolerance    Maximum heading error to be considered there.
     * @return {@link Command} that drives to the pose and then stops.
     */
    public Command driveToPose(Pose2d pose, Distance translationTolerance, Angle rotationTolerance) {
        return drive.driveToPose(pose, translationTolerance, rotationTolerance);
    }

    /** Point the wheels in an X so the robot resists being pushed, until canceled. */
    public Command lockPose() {
        return run(coroutine -> {
            while (true) {
                drive.lockPose();
                coroutine.yield();
            }
        }).named("Lock Pose");
    }

    /** Make the robot's current heading face away from the driver's alliance wall. */
    public Command zeroGyroWithAlliance() {
        return run(coroutine -> drive.resetOdometry(new Pose2d(getPose().getTranslation(), Field.isRed() ? Rotation2d.k180deg : Rotation2d.ZERO)))
            .named("Zero Gyro");
    }

    /** Reset odometry to a blue pose, flipped for the red alliance. */
    public Command resetOdometryCommand(Pose2d bluePose) {
        return run(coroutine -> drive.resetOdometry(Field.forAlliance(bluePose))).named("Reset Odometry");
    }

    public void stop() {
        drive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
    }

    /**
     * Underlying YAMS {@link SwerveDrive}, for building a {@link SwerveInputStream}.
     *
     * @return {@link SwerveDrive} driven by this mechanism.
     */
    public SwerveDrive getSwerveDrive() {
        return drive;
    }

    /** Current estimated pose of the robot, blue alliance origin. */
    public Pose2d getPose() {
        return drive.getPose();
    }

    /** Distance from the robot to this alliance's hub, in meters. */
    public double distanceToHub() {
        return Field.distanceToHub(getPose());
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        for (Vision camera : cameras) {
            camera.getMeasurement(getPose()).ifPresent(estimate -> drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds));
        }
        distanceToHubPublisher.set(distanceToHub());
        drive.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        drive.simIterate();
    }
}
