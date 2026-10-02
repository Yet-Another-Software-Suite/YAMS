// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.mechanisms;

import static first.robot.Constants.SwerveConstants.*;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.DriveConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.Field;
import first.robot.Ports;
import first.robot.Vision;
import java.util.List;
import java.util.function.DoubleSupplier;
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
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.commands3.swerve.SwerveInputStream;
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
    private final OnboardIMU gyro = new OnboardIMU(MountOrientation.FLAT);
    private final SwerveDrive drive;
    private final SwerveInputStream input;
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
            .withGyro(() -> Radians.of(gyro.getYawRadians()))
            // The original started the robot on the red side of the field, facing the blue wall.
            .withStartingPose(new Pose2d(13, 4, Rotation2d.k180deg))
            .withMaximumChassisSpeed(DriveConstants.kMaxSpeed, DriveConstants.kMaxAngularSpeed)
            .withMaximumModuleSpeed(DriveConstants.kMaxSpeed)
            // Heading PID from the YAGSL controller properties, used to aim at the hub.
            .withRotationController(new PIDController(kHeadingKP, 0, kHeadingKD))
            .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
        drive = new SwerveDrive(config);
        // The one drive input: field relative from the driver's alliance wall, with the original's
        // deadband and rotation scale. Drive commands set its sticks and modes every loop.
        input = new SwerveInputStream(drive)
            .withMaximumLinearVelocity(DriveConstants.kMaxSpeed)
            .withMaximumAngularVelocity(DriveConstants.kMaxAngularSpeed)
            .withDeadband(OperatorConstants.kDeadband)
            .withScaleRotation(DriveConstants.kRotationScale)
            .withAllianceRelativeControl(true);
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
            .withIdleMode(MotorMode.BRAKE)
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
            .withIdleMode(MotorMode.BRAKE)
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
     * Drive from the driver's sticks until canceled.
     *
     * @param forward          Stick input away from the driver, in [-1, 1].
     * @param left             Stick input to the driver's left, in [-1, 1].
     * @param rotation         Counterclockwise rotation stick input, in [-1, 1].
     * @param translationScale Scale on the translation sticks.
     * @param aimAtHub         Face the hub instead of rotating from the stick.
     * @param name             Command name.
     */
    public Command drive(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier rotation, double translationScale,
                         boolean aimAtHub, String name) {
        return run(coroutine -> {
            input.reset();
            while (true) {
                input.withTranslation(forward.getAsDouble(), left.getAsDouble())
                    .withRotation(rotation.getAsDouble())
                    .withScaleTranslation(translationScale)
                    .withAimTarget(new Pose2d(Field.hub(), Rotation2d.ZERO))
                    .withAim(aimAtHub);
                drive.setFieldRelativeChassisSpeeds(input.get());
                coroutine.yield();
            }
        }).named(name);
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
