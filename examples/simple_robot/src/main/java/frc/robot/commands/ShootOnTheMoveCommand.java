package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import yams.mechanisms.swerve.utility.SwerveInputStream;


public class ShootOnTheMoveCommand extends Command
{

  /**
   * The velocity scalar is used to compensate for the robot's velocity. This must be tuned and found empircially.
   */
  private final double velocityScalar    = 0.1;
  private final Angle  setpointTolerance = Degrees.of(1);
  private final SwerveSubsystem        swerveSubsystem;
  private final SwerveInputStream      inputStream;
  private final Pose2d                 targetPose;
  private final AngularVelocity        maxProfiledVelocity     = RotationsPerSecond.of(3);
  private final AngularAcceleration    maxProfiledAcceleration = RotationsPerSecondPerSecond.of(3);
  private final ProfiledPIDController  pidController           = new ProfiledPIDController(1,
                                                                                           0,
                                                                                           0,
                                                                                           new Constraints(
                                                                                               maxProfiledVelocity.in(
                                                                                                   RadiansPerSecond),
                                                                                               maxProfiledAcceleration.in(
                                                                                                   RadiansPerSecondPerSecond)));
  private final SimpleMotorFeedforward feedforward             = new SimpleMotorFeedforward(0, 0, 0);

  /**
   * Estimate the pose in which you will be shooting from, taking into account the current robot velocity.
   *
   * @param currentPose                {@link Pose2d} of the robot.
   * @param fieldOrientedChassisSpeed  Field oriented {@link ChassisSpeeds} of the robot.
   * @param velocityCompensationScalar Scalar applied to the chassis speeds to compensate for robot velocity.
   * @return Estimated {@link Pose2d} of the robot, based off the {@link ChassisSpeeds} multiplied by the scalar.
   */
  public static Pose2d estimatePose(Pose2d currentPose, ChassisSpeeds fieldOrientedChassisSpeed,
                                    double velocityCompensationScalar)
  {
    var deltaSpeed       = fieldOrientedChassisSpeed.times(velocityCompensationScalar);
    var deltaTranslation = new Translation2d(deltaSpeed.vxMetersPerSecond, deltaSpeed.vyMetersPerSecond);
    return new Pose2d(currentPose.getTranslation().plus(deltaTranslation),
                      currentPose.getRotation());
  }

  /**
   * Get the angle between two poses, useful to know where to point.
   *
   * @param currentPose Velocity compensated current pose.
   * @param targetPose  Target pose to point towards.
   * @return {@link Angle} between the two poses, to point to.
   */
  public static Angle getAngleTowardsPose(Pose2d currentPose, Pose2d targetPose)
  {
    return targetPose.getTranslation().minus(currentPose.getTranslation()).getAngle().getMeasure();
  }

  public ShootOnTheMoveCommand(SwerveSubsystem swerveSubsystem, SwerveInputStream inputStream, Pose2d targetPose)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.inputStream = inputStream;
    this.targetPose = targetPose;
    pidController.setTolerance(setpointTolerance.in(Radians));
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.swerveSubsystem);
  }

  @Override
  public void initialize()
  {
    pidController.reset(swerveSubsystem.getPose().getRotation().getRadians(),
                        swerveSubsystem.getFieldOrientedChassisSpeed().omegaRadiansPerSecond);
  }

  @Override
  public void execute()
  {
    var compensatedPose = estimatePose(swerveSubsystem.getPose(),
                                       swerveSubsystem.getFieldOrientedChassisSpeed(),
                                       velocityScalar);
    var setpointAngle = getAngleTowardsPose(compensatedPose, targetPose);
    var setpoint      = setpointAngle.in(Radians);
    var output = pidController.calculate(compensatedPose.getRotation().getRadians(),
                                         new State(setpoint, 0));
    var feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    var originalSpeed     = this.inputStream.get();
    originalSpeed.omegaRadiansPerSecond = output + feedforwardOutput;
    swerveSubsystem.driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(originalSpeed,
                                                                                   new Rotation2d(swerveSubsystem.getGyroAngle())));
    if (pidController.atGoal())
    {
      // TODO: Allow firing
    }
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {

  }
}
