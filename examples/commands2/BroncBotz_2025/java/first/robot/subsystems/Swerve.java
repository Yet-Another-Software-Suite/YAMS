// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.subsystems;

import static first.robot.Constants.SwerveConstants.*;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.DriveToPose;
import first.robot.Constants.OperatorConstants;
import first.robot.Ports;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.hardware.rotation.AnalogEncoder;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.commands2.swerve.SwerveInputStream;
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
public class Swerve extends SubsystemBase {
    private final SwerveDrive drive;
    private final Vision vision;

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
            .withGyro(gyro.getYaw().asSupplier())
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
            .withIdleMode(MotorMode.BRAKE)
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
            .withIdleMode(MotorMode.BRAKE)
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
     * Create the driver input stream. The original passed it to a robot relative drive, so the sticks
     * drive relative to the robot; alliance relative control can be toggled on to flip them for the
     * red alliance.
     *
     * @param forward          Stick input toward the robot's front, in [-1, 1].
     * @param left             Stick input toward the robot's left, in [-1, 1].
     * @param rotation         Counterclockwise rotation stick input, in [-1, 1].
     * @param translationScale Scale applied to the translation sticks.
     * @param allianceRelative Flip the sticks for the red alliance while true.
     */
    public SwerveInputStream createDriverInput(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier rotation,
                                               double translationScale, BooleanSupplier allianceRelative) {
        return new SwerveInputStream(drive, forward, left, rotation)
            .withMaximumLinearVelocity(kMaxSpeed)
            .withMaximumAngularVelocity(kMaxAngularSpeed)
            .withDeadband(OperatorConstants.kDeadband)
            .withScaleTranslation(translationScale)
            .withScaleRotation(OperatorConstants.kRotationScale)
            .withAllianceRelativeControl(allianceRelative);
    }

    /** Drive with robot relative speeds until interrupted. */
    public Command driveRobotRelative(Supplier<ChassisVelocities> speeds) {
        return run(() -> drive.setRobotRelativeChassisSpeeds(speeds.get()));
    }

    /**
     * Drive to a pose, ending once within the original's tolerances. Like the original's profiled
     * controller, the speed is capped at 1.3 m/s and 90 deg/s.
     *
     * @param pose Pose to drive to, blue alliance origin.
     */
    public Command driveToPose(Supplier<Pose2d> pose) {
        return defer(() -> {
            final Pose2d target = pose.get();
            return startRun(
                () -> {
                    drive.resetTranslationPID();
                    drive.resetRotationPID();
                    drive.getField2d().getObject("target").setPose(target);
                },
                () -> drive.setRobotRelativeChassisSpeeds(limit(drive.driveToPoseSetpoint(target))))
                .until(() -> isNear(target))
                .finallyDo(this::stop);
        });
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

    /** Drive straight back, relative to the robot, until interrupted. */
    public Command backOff() {
        return run(() -> drive.setRobotRelativeChassisSpeeds(
            new ChassisVelocities(-DriveToPose.kBackOffSpeed.in(MetersPerSecond), 0, 0)))
            .finallyDo(this::stop);
    }

    /** Point the wheels in an X so the robot cannot be pushed, until interrupted. */
    public Command lockPose() {
        return run(drive::lockPose);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void stop() {
        drive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
    }

    @Override
    public void periodic() {
        vision.update();
        drive.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        drive.simIterate();
    }
}
