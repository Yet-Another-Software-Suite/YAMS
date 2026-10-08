// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static first.robot.Constants.SwerveConstants.*;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.DriveToPose;
import first.robot.Ports;
import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.rotation.AnalogEncoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
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
 * Swerve drivetrain built with YAMS: four NEO / SPARK MAX modules with Thrifty absolute encoders and
 * a Pigeon 2 (the original had a Redux Canandgyro). The original built the same drivetrain with YAGSL from the deploy/swerve JSON
 * files; those values now live in {@code Constants.SwerveConstants}.
 */
public class Swerve implements Mechanism {
    private final SwerveDrive drive;
    private final Vision vision;
    /** Active driver input, set by the teleop opmode. */
    private SwerveInputStream inputStream;

    public Swerve() {
        final SwerveModule frontLeft = createModule("frontleft", Ports.kFrontLeftDrive, Ports.kFrontLeftAngle,
            Ports.kFrontLeftEncoder, kFrontLeftEncoderOffset, new Translation2d(kModuleOffset, kModuleOffset));
        final SwerveModule frontRight = createModule("frontright", Ports.kFrontRightDrive, Ports.kFrontRightAngle,
            Ports.kFrontRightEncoder, kFrontRightEncoderOffset, new Translation2d(kModuleOffset, kModuleOffset.unaryMinus()));
        final SwerveModule backLeft = createModule("backleft", Ports.kBackLeftDrive, Ports.kBackLeftAngle,
            Ports.kBackLeftEncoder, kBackLeftEncoderOffset, new Translation2d(kModuleOffset.unaryMinus(), kModuleOffset));
        final SwerveModule backRight = createModule("backright", Ports.kBackRightDrive, Ports.kBackRightAngle,
            Ports.kBackRightEncoder, kBackRightEncoderOffset,
            new Translation2d(kModuleOffset.unaryMinus(), kModuleOffset.unaryMinus()));

        // The Pigeon 2 reports yaw counterclockwise positive, without wrapping.
        final Pigeon2 gyro = new Pigeon2(Ports.kPigeon, Ports.kCTRECANBus);

        final SwerveDriveConfig config = new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
            .withGyro(gyro::getRotation3d)
            .withStartingPose(kStartingPose)
            .withMaximumModuleSpeed(kMaxSpeed)
            // Drive to pose gains from the original's AlignmentConstants.
            .withTranslationController(new PIDController(DriveToPose.kTranslationP, 0, 0))
            .withRotationController(new PIDController(DriveToPose.kRotationP, 0, 0))
            .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
        drive = new SwerveDrive(config);
        vision = new Vision(drive);
    }

    private SwerveModule createModule(String name, int driveId, int angleId, int encoderChannel, Angle encoderOffset,
                                      Translation2d location) {
        final SparkMax driveMotor = new SparkMax(Ports.kCANBus, driveId, MotorType.kBrushless);
        final SparkMax angleMotor = new SparkMax(Ports.kCANBus, angleId, MotorType.kBrushless);
        // Thrifty absolute encoders are analog, read as one rotation over the input range.
        final AnalogEncoder encoder = new AnalogEncoder(encoderChannel);

        final SmartMotorControllerConfig driveConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kDriveGearRatio))
            .withWheelDiameter(kWheelDiameter)
            .withClosedLoopController(kDriveKP, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, kDriveKV))
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(kDriveCurrentLimit)
            .withOpenLoopRampRate(kRampRate)
            .withClosedLoopRampRate(kRampRate)
            .withMotorInverted(true)
            .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

        final SmartMotorControllerConfig angleConfig = new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kAngleGearRatio))
            .withClosedLoopController(kAngleKP, 0, 0)
            .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
            .withZeroPower(MotorMode.BRAKE)
            .withStatorCurrentLimit(kAngleCurrentLimit)
            .withOpenLoopRampRate(kRampRate)
            .withClosedLoopRampRate(kRampRate)
            .withMotorInverted(false)
            .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

        final SmartMotorController driveController = new SparkWrapper(driveMotor, DCMotor.getNEO(1), driveConfig);
        final SmartMotorController angleController = new SparkWrapper(angleMotor, DCMotor.getNEO(1), angleConfig);

        return new SwerveModule(new SwerveModuleConfig(driveController, angleController)
            // The absolute encoder seeds the angle motor's encoder; the SPARK closes the loop on that.
            .withAbsoluteEncoder(() -> Rotations.of(encoder.get()))
            .withAbsoluteEncoderOffset(encoderOffset)
            .withLocation(location)
            .withOptimization(true)
            .withTelemetry(name, TelemetryVerbosity.HIGH));
    }

    /** Current estimated pose of the robot, blue alliance origin. */
    public Pose2d getPose() {
        return drive.getPose();
    }

    /**
     * Drive with field relative {@link ChassisVelocities}. Runs until canceled.
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
     * Drive with the active {@link SwerveInputStream}. A stream set while this runs takes effect on the next loop.
     * Runs until canceled.
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
     * Underlying YAMS {@link SwerveDrive}, for building a {@link SwerveInputStream}.
     *
     * @return {@link SwerveDrive} driven by this mechanism.
     */
    public SwerveDrive getSwerveDrive() {
        return drive;
    }

    /**
     * Drive to a pose, ending once within the original's tolerances. Like the original's profiled
     * controller, the speed is capped at 1.3 m/s and 90 deg/s. With no pose it does nothing.
     *
     * @param pose Pose to drive to, blue alliance origin, read when the command starts.
     */
    public Command driveToPose(Supplier<Optional<Pose2d>> pose) {
        return run(coroutine -> {
            final Optional<Pose2d> target = pose.get();
            if (target.isEmpty()) {
                return;
            }
            drive.resetTranslationPID();
            drive.resetRotationPID();
            drive.getField2d().getObject("target").setPose(target.get());
            while (!isNear(target.get())) {
                drive.setRobotRelativeChassisSpeeds(limit(drive.driveToPoseSetpoint(target.get())));
                coroutine.yield();
            }
            stop();
        }).whenCanceled(this::stop).named("Drive to Pose");
    }

    private boolean isNear(Pose2d target) {
        return drive.getDistanceFromPose(target).lte(DriveToPose.kTranslationTolerance)
            && Math.abs(drive.getAngleDifferenceFromPose(target).in(Rotations)) <= DriveToPose.kRotationTolerance.in(Rotations);
    }

    private static ChassisVelocities limit(ChassisVelocities speeds) {
        final double maxSpeed = DriveToPose.kMaxSpeed.in(MetersPerSecond);
        final double speed = Math.hypot(speeds.vx, speeds.vy);
        final double scale = speed > maxSpeed ? maxSpeed / speed : 1;
        final double maxOmega = DriveToPose.kMaxAngularSpeed.in(RadiansPerSecond);
        return new ChassisVelocities(speeds.vx * scale, speeds.vy * scale, Math.clamp(speeds.omega, -maxOmega, maxOmega));
    }

    /** Drive straight back, relative to the robot, until canceled. */
    public Command backOff() {
        return run(coroutine -> {
            while (true) {
                drive.setRobotRelativeChassisSpeeds(new ChassisVelocities(-DriveToPose.kBackOffSpeed.in(MetersPerSecond), 0, 0));
                coroutine.yield();
            }
        }).whenCanceled(this::stop).named("Back Off");
    }

    /** Point the wheels in an X so the robot cannot be pushed, until canceled. */
    public Command lockPose() {
        return run(coroutine -> {
            while (true) {
                drive.lockPose();
                coroutine.yield();
            }
        }).named("Lock Pose");
    }

    public Command stopCommand() {
        return run(coroutine -> stop()).named("Stop Driving");
    }

    private void stop() {
        drive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
    }

    /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
    public void periodic() {
        vision.update();
        drive.updateTelemetry();
    }

    /** Called from {@code Robot.simulationPeriodic()}. */
    public void simulationPeriodic() {
        drive.simIterate();
    }
}
