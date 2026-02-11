package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import java.util.List;
import java.util.function.Supplier;


/**
 * Largely written by Eeshwar based off their blog at https://blog.eeshwark.com/robotblog/shooting-on-the-fly
 */
public class ShootOnTheMoveCommand extends Command
{

  // Subsystems
  private final TurretSubsystem   turretSubsystem;
  private final HoodSubsystem     hoodSubsystem;
  private final FlywheelSubsystem flywheelSubsystem;

  /**
   * Current robot pose. (Blue-alliance)
   */
  private final Supplier<Pose2d>        robotPose;
  /**
   * Current field-oriented chassis speeds.
   */
  private final Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds;
  /**
   * Pose to shoot at.
   */
  private final Pose2d                  goalPose;

  // Tuned Constants
  double totalExitVelocity = 15.0; // m/s
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double                     latency      = 0.15;
  /**
   * Maps Distance to RPM
   */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  /**
   * Shoot on the move command to always have the turret ready to fire.
   *
   * @param turret                     Turret subsystem
   * @param hood                       Hood subsystem
   * @param flyWheel                   Flywheel subsystem
   * @param currentPose                Current robot pose.
   * @param fieldOrientedChassisSpeeds Current field-oriented chassis speeds.
   * @param goal                       Goal to shoot at.
   */
  public ShootOnTheMoveCommand(TurretSubsystem turret, HoodSubsystem hood, FlywheelSubsystem flyWheel,
                               Supplier<Pose2d> currentPose, Supplier<ChassisSpeeds> fieldOrientedChassisSpeeds,
                               Pose2d goal)
  {
    turretSubsystem = turret;
    hoodSubsystem = hood;
    flywheelSubsystem = flyWheel;
    robotPose = currentPose;
    this.fieldOrientedChassisSpeeds = fieldOrientedChassisSpeeds;
    this.goalPose = goal;

    // Test Results
    for (var entry : List.of(Pair.of(Meters.of(1), RPM.of((1000))),
                             Pair.of(Meters.of(2), RPM.of(2000)),
                             Pair.of(Meters.of(3), RPM.of(3000)))
    )
    {shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));}

    setName("Shoot on the move");
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = fieldOrientedChassisSpeeds.get();
    // 1. LATENCY COMP
    Translation2d futurePos = robotPose.get().getTranslation().plus(
        new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond).times(latency)
                                                                   );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = goalPose.getTranslation();
    Translation2d targetVec    = goalLocation.minus(futurePos);
    double        dist         = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vxMetersPerSecond, robotSpeed.vyMetersPerSecond);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    double turretAngle        = shotVec.getAngle().getDegrees();
    double newHorizontalSpeed = shotVec.getNorm();

    // 6. SOLVE FOR NEW PITCH/RPM
    // Assuming constant total exit velocity, variable hood:
    // Clamp to avoid domain errors if we need more speed than possible
    double ratio    = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
    double newPitch = Math.acos(ratio);

    // 7. SET OUTPUTS
    turretSubsystem.setAngleDirect(Degrees.of(turretAngle));
    hoodSubsystem.setAngleDirect(Radians.of(newPitch));
    flywheelSubsystem.setRPMDirect(MetersPerSecond.of(totalExitVelocity));
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
