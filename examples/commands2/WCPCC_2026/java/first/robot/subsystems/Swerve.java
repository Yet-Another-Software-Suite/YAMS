// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.subsystems;

import static first.robot.Constants.SwerveConstants.*;
import static org.wpilib.units.Units.Newtons;
import static org.wpilib.units.Units.Rotations;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.Driving;
import first.robot.util.GeometryUtil;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import first.robot.Ports;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Force;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.config.SwerveDriveConfig;
import yams.commands2.swerve.SwerveDrive;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

/**
 * Swerve drivetrain built with YAMS: four Kraken X60 modules with fused CANcoders and a Pigeon 2.
 *
 * <p>The WCP code extended CTRE's generated swerve drivetrain and drove it with swerve requests.
 * Teleop driving now goes through a YAMS {@link SwerveInputStream}, which handles the deadband,
 * response curve, alliance relative control, heading hold, and aiming; this subsystem adds Choreo
 * path following and vision.
 */
public class Swerve extends SubsystemBase {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.ZERO;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    private final SwerveDrive drive;

    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;
    private Rotation2d operatorForwardDirection = kBlueAlliancePerspectiveRotation;

    private final PIDController pathXController = new PIDController(10, 0, 0);
    private final PIDController pathYController = new PIDController(10, 0, 0);
    private final PIDController pathThetaController = new PIDController(7, 0, 0);

    public Swerve() {
        pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

        final SwerveModule frontLeft = createModule("frontleft", Ports.kFrontLeftDrive, Ports.kFrontLeftSteer,
            Ports.kFrontLeftEncoder, kFrontLeftEncoderOffset, kInvertLeftSide,
            new Translation2d(kModuleOffset, kModuleOffset));
        final SwerveModule frontRight = createModule("frontright", Ports.kFrontRightDrive, Ports.kFrontRightSteer,
            Ports.kFrontRightEncoder, kFrontRightEncoderOffset, kInvertRightSide,
            new Translation2d(kModuleOffset, kModuleOffset.unaryMinus()));
        final SwerveModule backLeft = createModule("backleft", Ports.kBackLeftDrive, Ports.kBackLeftSteer,
            Ports.kBackLeftEncoder, kBackLeftEncoderOffset, kInvertLeftSide,
            new Translation2d(kModuleOffset.unaryMinus(), kModuleOffset));
        final SwerveModule backRight = createModule("backright", Ports.kBackRightDrive, Ports.kBackRightSteer,
            Ports.kBackRightEncoder, kBackRightEncoderOffset, kInvertRightSide,
            new Translation2d(kModuleOffset.unaryMinus(), kModuleOffset.unaryMinus()));

        final Pigeon2 pigeon = new Pigeon2(Ports.kPigeon, Ports.kCANivoreCANBus);

        final SwerveDriveConfig config = (SwerveDriveConfig) new SwerveDriveConfig(this, frontLeft, frontRight, backLeft, backRight)
            .withGyro(pigeon.getYaw().asSupplier())
            .withStartingPose(Pose2d.ZERO)
            .withMaximumModuleSpeed(kSpeedAt12Volts)
            // Heading PID from the original drive requests; SwerveInputStream uses it to hold, snap,
            // and aim the robot's heading.
            .withRotationController(new PIDController(5, 0, 0))
            .withTelemetry("Swerve", TelemetryVerbosity.HIGH);
        drive = new SwerveDrive(config);
    }

    private SwerveModule createModule(String name, int driveId, int steerId, int encoderId, Angle encoderOffset,
                                      boolean driveInverted, Translation2d location) {
        final TalonFX driveMotor = new TalonFX(driveId, Ports.kCANivoreCANBus);
        final TalonFX steerMotor = new TalonFX(steerId, Ports.kCANivoreCANBus);
        final CANcoder encoder = new CANcoder(encoderId, Ports.kCANivoreCANBus);

        final SmartMotorControllerConfig driveConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kDriveGearRatio))
            .withWheelRadius(kWheelRadius)
            // Tuner X drive gains are per motor rotation; YAMS runs them per wheel rotation.
            .withClosedLoopController(kDriveKP * kDriveGearRatio, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, kDriveKV * kDriveGearRatio))
            .withIdleMode(MotorMode.BRAKE)
            // The slip current limits the drive stator current, as in the CTRE swerve API.
            .withStatorCurrentLimit(kSlipCurrent)
            .withMotorInverted(driveInverted)
            .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

        final SmartMotorControllerConfig steerConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withGearing(new MechanismGearing(kSteerGearRatio))
            .withClosedLoopController(kSteerKP, 0, kSteerKD)
            .withFeedforward(new SimpleMotorFeedforward(kSteerKS, kSteerKV))
            // Fused CANcoder feedback: the Talon closes the steering loop on the module angle.
            .withExternalEncoder(encoder)
            .withUseExternalFeedbackEncoder(true)
            .withExternalEncoderInverted(false)
            .withExternalEncoderZeroOffset(encoderOffset)
            .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
            .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
            .withIdleMode(MotorMode.BRAKE)
            .withStatorCurrentLimit(kSteerStatorCurrentLimit)
            .withMotorInverted(kSteerMotorInverted)
            .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

        final SmartMotorController driveController = new TalonFXWrapper(driveMotor, DCMotor.getKrakenX60(1), driveConfig);
        final SmartMotorController steerController = new TalonFXWrapper(steerMotor, DCMotor.getKrakenX60(1), steerConfig);

        return new SwerveModule(new SwerveModuleConfig(driveController, steerController)
            .withLocation(location)
            .withOptimization(true)
            .withTelemetry(name, TelemetryVerbosity.HIGH));
    }

    /** Current estimated pose of the robot, blue alliance origin. */
    public Pose2d getPose() {
        return drive.getPose();
    }

    public void resetPose(Pose2d pose) {
        drive.resetOdometry(pose);
    }

    /** Direction the operator considers forward: toward the opposing alliance wall. */
    public Rotation2d getOperatorForwardDirection() {
        return operatorForwardDirection;
    }

    /** Make the direction the robot is currently facing "forward" for field centric driving. */
    public void seedFieldCentric() {
        resetPose(new Pose2d(getPose().getTranslation(), operatorForwardDirection));
    }

    /**
     * Create a driver input stream: field centric from the operator's perspective, with WCP's
     * joystick deadband and a cubed response on both the translation and rotation axes.
     *
     * @param forward  Stick input away from the operator, in [-1, 1].
     * @param left     Stick input to the operator's left, in [-1, 1].
     * @param rotation Counterclockwise rotation stick input, in [-1, 1].
     * @return {@link SwerveInputStream} producing field relative {@link ChassisVelocities}.
     */
    public SwerveInputStream createDriverInput(DoubleSupplier forward, DoubleSupplier left, DoubleSupplier rotation) {
        return new SwerveInputStream(drive, forward, left, rotation)
            .withMaximumLinearVelocity(Driving.kMaxSpeed)
            .withMaximumAngularVelocity(Driving.kMaxRotationalRate)
            .withDeadband(Driving.kJoystickDeadband)
            .withCubeTranslationControllerAxis()
            .withCubeRotationControllerAxis()
            .withAllianceRelativeControl();
    }

    /** Drive with field relative speeds, e.g. from {@link #createDriverInput}. */
    public void driveFieldRelative(ChassisVelocities fieldRelativeSpeeds) {
        drive.setFieldRelativeChassisSpeeds(fieldRelativeSpeeds);
    }

    /** Command that drives with field relative speeds, e.g. from {@link #createDriverInput}. */
    public Command driveCommand(Supplier<ChassisVelocities> fieldRelativeSpeeds) {
        return run(() -> driveFieldRelative(fieldRelativeSpeeds.get()));
    }

    /**
     * Whether the robot is facing a field position within a tolerance.
     *
     * @param target    Field position, blue alliance origin.
     * @param tolerance Allowed heading error.
     */
    public boolean isFacing(Translation2d target, Angle tolerance) {
        final Pose2d pose = getPose();
        return target.minus(pose.getTranslation()).getAngle()
            .map(direction -> GeometryUtil.isNear(direction, pose.getRotation(), tolerance))
            .orElse(true);
    }

    /**
     * Creates a new auto factory for this drivetrain.
     *
     * @return AutoFactory for this drivetrain
     */
    public AutoFactory createAutoFactory() {
        return new AutoFactory(this::getPose, this::resetPose, this::followPath, true, this);
    }

    /**
     * Follows the given field-centric path sample with PID, plus Choreo's per-module force
     * feedforwards.
     *
     * @param sample Sample along the path to follow
     */
    public void followPath(SwerveSample sample) {
        final Pose2d pose = getPose();
        final ChassisVelocities targetSpeeds = sample.getChassisSpeeds();
        targetSpeeds.vx += pathXController.calculate(pose.getX(), sample.x);
        targetSpeeds.vy += pathYController.calculate(pose.getY(), sample.y);
        targetSpeeds.omega += pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

        final Rotation2d heading = pose.getRotation();
        final ChassisVelocities robotRelativeSpeeds = targetSpeeds.toRobotRelative(heading);
        drive.setRobotRelativeChassisSpeeds(robotRelativeSpeeds, getModuleForces(sample, robotRelativeSpeeds, heading));
    }

    /**
     * Convert Choreo's field relative X and Y force for each module into the single force YAMS takes,
     * along the direction each module is asked to drive. YAMS reverses it if optimization turns the
     * wheel around.
     */
    private Force[] getModuleForces(SwerveSample sample, ChassisVelocities robotRelativeSpeeds, Rotation2d heading) {
        final double[] fieldForcesX = sample.moduleForcesX();
        final double[] fieldForcesY = sample.moduleForcesY();
        final SwerveModuleVelocity[] moduleStates = drive.getStateFromRobotRelativeChassisSpeeds(robotRelativeSpeeds);
        final Force[] forces = new Force[moduleStates.length];
        for (int i = 0; i < moduleStates.length; i++) {
            final Translation2d robotRelativeForce = new Translation2d(fieldForcesX[i], fieldForcesY[i]).rotateBy(heading.unaryMinus());
            final Rotation2d moduleDirection = moduleStates[i].angle;
            forces[i] = Newtons.of(robotRelativeForce.getX() * moduleDirection.getCos()
                + robotRelativeForce.getY() * moduleDirection.getSin());
        }
        return forces;
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds,
                                     Matrix<N3, N1> visionMeasurementStdDevs) {
        drive.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }

    @Override
    public void periodic() {
        /*
         * Periodically try to apply the operator perspective.
         * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
         * This allows us to correct the perspective in case the robot code restarts mid-match.
         * Otherwise, only check and apply the operator perspective if the DS is disabled.
         * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
         */
        if (!m_hasAppliedOperatorPerspective || RobotState.isDisabled()) {
            MatchState.getAlliance().ifPresent(allianceColor -> {
                operatorForwardDirection = allianceColor == Alliance.RED
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation;
                if (!m_hasAppliedOperatorPerspective) {
                    seedFieldCentric();
                }
                m_hasAppliedOperatorPerspective = true;
            });
        }
        drive.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        drive.simIterate();
    }
}
