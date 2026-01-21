package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import yams.mechanisms.swerve.utility.SwerveInputStream;


public class AlignToGoal extends Command
{

  private final SwerveSubsystem        swerveSubsystem;
  private final ShooterSubsystem           shooterSubsystem;
  private final SwerveInputStream      inputStream;
  private final Pose2d                 targetPose;
  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency             = 0.15;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable        = new InterpolatingDoubleTreeMap();
  private final Angle                      setpointTolerance   = Degrees.of(1);
  private final AngularVelocity            maxProfiledVelocity = RotationsPerSecond.of(3);
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


  public AlignToGoal(SwerveSubsystem swerveSubsystem, ShooterSubsystem shooter, SwerveInputStream inputStream,
                     Pose2d targetPose)
  {
    this.swerveSubsystem = swerveSubsystem;
    this.shooterSubsystem = shooter;
    this.inputStream = inputStream;
    this.targetPose = targetPose;
    pidController.setTolerance(setpointTolerance.in(Radians));
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)

    // Test Results
    for (var entry : List.of(Pair.of(Meters.of(1), RPM.of((1000))),
                             Pair.of(Meters.of(2), RPM.of(2000)),
                             Pair.of(Meters.of(3), RPM.of(3000)))
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    addRequirements(this.swerveSubsystem, this.shooterSubsystem);
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
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = swerveSubsystem.getFieldOrientedChassisSpeed();
    // 1. LATENCY COMP
    Translation2d futurePos = swerveSubsystem.getPose().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                             );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = targetPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    Angle          turretAngle        = Degrees.of(shotVec.getAngle().getDegrees());
    LinearVelocity newHorizontalSpeed = MetersPerSecond.of(shotVec.getNorm());

    // 7. SET OUTPUTS
    var output = pidController.calculate(swerveSubsystem.getPose().getRotation().getRadians(),
                                         new State(turretAngle.in(Radians), 0));
    var feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    var originalSpeed     = this.inputStream.get();
    originalSpeed.omegaRadiansPerSecond = output + feedforwardOutput;
    swerveSubsystem.driveRobotRelative(() -> ChassisSpeeds.fromFieldRelativeSpeeds(originalSpeed,
                                                                                   new Rotation2d(swerveSubsystem.getGyroAngle())));
    shooterSubsystem.setRPM(newHorizontalSpeed);
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
