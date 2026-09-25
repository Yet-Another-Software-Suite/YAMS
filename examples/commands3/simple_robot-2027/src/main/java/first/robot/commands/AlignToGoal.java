// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RadiansPerSecondPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;

import org.wpilib.util.Pair;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.trajectory.TrapezoidProfile.State;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine;
import org.wpilib.command3.Mechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import java.util.List;
import java.util.Set;
import yams.core.mechanisms.swerve.utility.SwerveInputStream;

public class AlignToGoal implements Command {
  private final SwerveMechanism swerveMechanism;
  private final ShooterMechanism shooterMechanism;
  private final SwerveInputStream inputStream;
  private final Pose2d targetPose;

  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private final double latency = 0.15;

  /** Maps Distance to RPM */
  private final InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();

  private final Angle setpointTolerance = Degrees.of(1);
  private final AngularVelocity maxProfiledVelocity = RotationsPerSecond.of(3);
  private final AngularAcceleration maxProfiledAcceleration = RotationsPerSecondPerSecond.of(3);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(
          1,
          0,
          0,
          new Constraints(
              maxProfiledVelocity.in(RadiansPerSecond),
              maxProfiledAcceleration.in(RadiansPerSecondPerSecond)));
  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

  public AlignToGoal(
      SwerveMechanism swerveMechanism,
      ShooterMechanism shooter,
      SwerveInputStream inputStream,
      Pose2d targetPose) {
    this.swerveMechanism = swerveMechanism;
    this.shooterMechanism = shooter;
    this.inputStream = inputStream;
    this.targetPose = targetPose;
    pidController.setTolerance(setpointTolerance.in(Radians));

    // Test Results
    for (var entry :
        List.of(
            Pair.of(Meters.of(1), RPM.of((1000))),
            Pair.of(Meters.of(2), RPM.of(2000)),
            Pair.of(Meters.of(3), RPM.of(3000)))) {
      shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
    }
  }

  @Override
  public String name() {
    return "AlignToGoal";
  }

  @Override
  public Set<Mechanism> requirements() {
    return Set.of(swerveMechanism, shooterMechanism);
  }

  @Override
  public void run(Coroutine coroutine) {
    initialize();
    while (true) {
      execute();
      coroutine.yield();
    }
  }

  private void initialize()
  {
    pidController.reset(swerveMechanism.getPose().getRotation().getRadians(),
                        swerveMechanism.getFieldOrientedChassisSpeed().omega);
  }

  private void execute() {
    // Please look here for the original authors work!
    // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
    // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    // YASS did not come up with this
    // -------------------------------------------------------

    var robotSpeed = swerveMechanism.getFieldOrientedChassisSpeed();
    // 1. LATENCY COMP
    Translation2d futurePos = swerveMechanism.getPose().getTranslation().plus(
        new Translation2d(robotSpeed.vx, robotSpeed.vy).times(latency)
                                                                             );

    // 2. GET TARGET VECTOR
    Translation2d goalLocation = targetPose.getTranslation();
    Translation2d targetVec = goalLocation.minus(futurePos);
    double dist = targetVec.getNorm();

    // 3. CALCULATE IDEAL SHOT (Stationary)
    // Note: This returns HORIZONTAL velocity component
    double idealHorizontalSpeed = shooterTable.get(dist);

    // 4. VECTOR SUBTRACTION
    Translation2d robotVelVec = new Translation2d(robotSpeed.vx, robotSpeed.vy);
    Translation2d shotVec     = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

    // 5. CONVERT TO CONTROLS
    Angle turretAngle = Degrees.of(shotVec.getAngle().orElse(Rotation2d.ZERO).getDegrees());
    LinearVelocity newHorizontalSpeed = MetersPerSecond.of(shotVec.getNorm());

    // 7. SET OUTPUTS
    var output =
        pidController.calculate(
            swerveMechanism.getPose().getRotation().getRadians(),
            new State(turretAngle.in(Radians), 0));
    var feedforwardOutput = feedforward.calculate(pidController.getSetpoint().velocity);
    var originalSpeed     = this.inputStream.get();
    originalSpeed.omega = output + feedforwardOutput;
    swerveMechanism.setRobotRelativeChassisSpeedsSetpoint(originalSpeed.toRobotRelative(new Rotation2d(swerveMechanism.getGyroAngle())));
    shooterMechanism.setRPM(newHorizontalSpeed);
  }
}
