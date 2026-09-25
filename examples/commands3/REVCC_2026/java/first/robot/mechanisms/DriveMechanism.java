// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Centimeters;
import static org.wpilib.units.Units.Radians;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.OIConstants;
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
import yams.commands3.swerve.SwerveInputStream;
import yams.core.mechanisms.swerve.SwerveModule;
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
  private final SwerveInputStream m_input;

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
    // The one driver input. Drive commands set its sticks every loop; the sticks are scaled to the
    // maximum chassis speeds configured above.
    m_input = new SwerveInputStream(m_drive)
        .withMaximumLinearVelocity(DriveConstants.kMaxSpeed)
        .withMaximumAngularVelocity(DriveConstants.kMaxAngularSpeed)
        .withDeadband(OIConstants.kDriveDeadband);
  }

  /** Reset the drive input: sticks at zero and field relative translation. */
  public void resetDriveInput()
  {
    m_input.reset().withRobotRelative(false);
  }

  /**
   * Set the driver's stick inputs. The sticks are scaled to the maximum chassis speeds configured
   * above.
   *
   * @param forward  Forward stick input, [-1, 1].
   * @param left     Left stick input, [-1, 1].
   * @param rotation Counterclockwise rotation stick input, [-1, 1].
   */
  public void setDriveInput(double forward, double left, double rotation)
  {
    m_input.withTranslation(forward, left).withRotation(rotation);
  }

  /**
   * Drive the translation sticks relative to the field or to the robot.
   *
   * @param fieldRelative Whether the translation sticks are relative to the field.
   */
  public void setFieldRelative(boolean fieldRelative)
  {
    m_input.withRobotRelative(!fieldRelative);
  }

  /** Drive from the drive input set with the methods above. Call once per loop. */
  public void driveFromInput()
  {
    // The stream always outputs field relative speeds; robot relative sticks are converted.
    m_drive.setFieldRelativeChassisSpeeds(m_input.get());
  }

  /** Sets the wheels into an X formation to prevent movement. Call once per loop to hold it. */
  public void lockWheels()
  {
    m_drive.lockPose();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading()
  {
    m_drive.zeroGyro();
  }

  /**
   * Command to drive to a field relative pose, ending once the robot is within 5 cm and 3 degrees.
   * The modules are stopped when it ends or is canceled.
   *
   * @param pose Target pose.
   */
  public Command driveToPoseCommand(Pose2d pose)
  {
    return m_drive.driveToPose(pose, Centimeters.of(5), Degrees.of(3));
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
