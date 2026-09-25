// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import first.robot.Constants.DriveConstants;
import java.util.function.DoubleSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.imu.OnboardIMU;
import org.wpilib.hardware.imu.OnboardIMU.MountOrientation;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import yams.commands3.config.SwerveDriveConfig;
import yams.commands3.swerve.SwerveDrive;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Swerve drive for the 2026 REV ION Starter Bot: four REV EasySwerve modules and the Systemcore
 * onboard IMU. YAMS {@link SwerveDrive} replaces the hand-written kinematics, odometry, and
 * desaturation from the REV code.
 */
public class DriveMechanism implements Mechanism
{
  // The gyro sensor. The REV code used an ADIS16470, which does not exist on Systemcore; the
  // onboard IMU is its replacement.
  private final OnboardIMU m_gyro = new OnboardIMU(MountOrientation.FLAT);

  private final SwerveDrive m_drive;

  /** Creates a new DriveMechanism. */
  public DriveMechanism()
  {
    // Module locations use the WPILib coordinate frame: +X toward the front, +Y toward the left.
    Translation2d frontLeftLocation  = new Translation2d(DriveConstants.kWheelBase.div(2),
                                                         DriveConstants.kTrackWidth.div(2));
    Translation2d frontRightLocation = new Translation2d(DriveConstants.kWheelBase.div(2),
                                                         DriveConstants.kTrackWidth.div(-2));
    Translation2d rearLeftLocation   = new Translation2d(DriveConstants.kWheelBase.div(-2),
                                                         DriveConstants.kTrackWidth.div(2));
    Translation2d rearRightLocation  = new Translation2d(DriveConstants.kWheelBase.div(-2),
                                                         DriveConstants.kTrackWidth.div(-2));

    // Create EasySwerveModules
    SwerveModule frontLeft = EasySwerveModule.create(this, "frontleft", frontLeftLocation,
                                                     DriveConstants.kFrontLeftDrivingCanId,
                                                     DriveConstants.kFrontLeftTurningCanId,
                                                     DriveConstants.kFrontLeftChassisAngularOffset,
                                                     DriveConstants.kFrontLeftDrivingMotorOnBottom,
                                                     DriveConstants.kFrontLeftTurningMotorOnBottom);
    SwerveModule frontRight = EasySwerveModule.create(this, "frontright", frontRightLocation,
                                                      DriveConstants.kFrontRightDrivingCanId,
                                                      DriveConstants.kFrontRightTurningCanId,
                                                      DriveConstants.kFrontRightChassisAngularOffset,
                                                      DriveConstants.kFrontRightDrivingMotorOnBottom,
                                                      DriveConstants.kFrontRightTurningMotorOnBottom);
    SwerveModule rearLeft = EasySwerveModule.create(this, "rearleft", rearLeftLocation,
                                                    DriveConstants.kRearLeftDrivingCanId,
                                                    DriveConstants.kRearLeftTurningCanId,
                                                    DriveConstants.kBackLeftChassisAngularOffset,
                                                    DriveConstants.kRearLeftDrivingMotorOnBottom,
                                                    DriveConstants.kRearLeftTurningMotorOnBottom);
    SwerveModule rearRight = EasySwerveModule.create(this, "rearright", rearRightLocation,
                                                     DriveConstants.kRearRightDrivingCanId,
                                                     DriveConstants.kRearRightTurningCanId,
                                                     DriveConstants.kBackRightChassisAngularOffset,
                                                     DriveConstants.kRearRightDrivingMotorOnBottom,
                                                     DriveConstants.kRearRightTurningMotorOnBottom);

    SwerveDriveConfig config = (SwerveDriveConfig) new SwerveDriveConfig(this, frontLeft, frontRight, rearLeft, rearRight)
        .withGyro(() -> Radians.of(m_gyro.getYawRadians()))
        .withGyroInverted(DriveConstants.kGyroReversed)
        .withStartingPose(Pose2d.ZERO)
        // Caps chassis speeds and desaturates module speeds, like the REV drive() method did.
        .withMaximumChassisSpeed(DriveConstants.kMaxSpeed, DriveConstants.kMaxAngularSpeed)
        .withMaximumModuleSpeed(DriveConstants.kMaxSpeed)
        // Used by driveToPose() in the autonomous routine; same gains as the REV AutoConstants.
        .withTranslationController(new PIDController(1, 0, 0))
        .withRotationController(new PIDController(1, 0, 0))
        .withTelemetry("Drive", TelemetryVerbosity.HIGH);
    m_drive = new SwerveDrive(config);
  }

  /**
   * Joystick input stream for teleop driving. Inputs are in [-1, 1] and are scaled to the maximum
   * chassis speeds configured above.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot    Angular rate of the robot.
   * @param deadband Joystick deadband.
   * @return {@link SwerveInputStream} producing field relative {@link ChassisVelocities}.
   */
  public SwerveInputStream getInputStream(DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier rot,
                                          double deadband)
  {
    return new SwerveInputStream(m_drive, xSpeed, ySpeed, rot)
        .withMaximumLinearVelocity(DriveConstants.kMaxSpeed)
        .withMaximumAngularVelocity(DriveConstants.kMaxAngularSpeed)
        .withDeadband(deadband);
  }

  /**
   * Command to drive the robot using joystick info.
   *
   * @param speeds        Chassis speeds, e.g. from {@link #getInputStream}.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public Command driveCommand(SwerveInputStream speeds, boolean fieldRelative)
  {
    return runRepeatedly(() -> {
      if (fieldRelative)
      {
        m_drive.setFieldRelativeChassisSpeeds(speeds.get());
      } else
      {
        m_drive.setRobotRelativeChassisSpeeds(speeds.get());
      }
    }).named(fieldRelative ? "Drive.FieldRelative" : "Drive.RobotRelative");
  }

  /**
   * Command to drive to a field relative pose, ending once the robot is within 5 cm and 3 degrees.
   * The modules are stopped when it ends or is canceled.
   *
   * @param pose Target pose.
   */
  public Command driveToPoseCommand(Pose2d pose)
  {
    return run(coroutine -> {
      coroutine.await(m_drive.driveToPose(pose).until(() -> isAtPose(pose)).withAutomaticName());
      stop();
    }).whenCanceled(this::stop)
      .named("Drive.DriveToPose" + pose);
  }

  /** Whether the robot is within 5 cm and 3 degrees of the pose. */
  private boolean isAtPose(Pose2d pose)
  {
    return m_drive.getDistanceFromPose(pose).lt(Meters.of(0.05))
           && Math.abs(m_drive.getAngleDifferenceFromPose(pose).in(Degrees)) < 3;
  }

  /** Stop all modules. */
  public void stop()
  {
    m_drive.setRobotRelativeChassisSpeeds(new ChassisVelocities());
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose()
  {
    return m_drive.getPose();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose)
  {
    m_drive.resetOdometry(pose);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public Command setXCommand()
  {
    return runRepeatedly(m_drive::lockPose).named("Drive.SetX");
  }

  /** Zeroes the heading of the robot. */
  public Command zeroHeadingCommand()
  {
    return run(coroutine -> m_drive.zeroGyro()).named("Drive.ZeroHeading");
  }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading()
  {
    return m_drive.getPose().getRotation().getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate()
  {
    return m_gyro.getGyroRateZ() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  /** Updates odometry and publishes the drive telemetry, including the field widget. */
  public void updateTelemetry()
  {
    m_drive.updateTelemetry();
  }

  /** Steps the drive physics simulation. */
  public void simIterate()
  {
    m_drive.simIterate();
  }
}
