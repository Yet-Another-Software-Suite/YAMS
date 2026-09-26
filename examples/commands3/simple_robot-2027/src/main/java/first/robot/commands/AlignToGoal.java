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
import org.wpilib.command3.button.CommandNiDsXboxController;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import java.util.List;

/** Factory for a command that rotates the drivetrain toward a goal and spins the shooter while driving. */
public final class AlignToGoal {
  // Tuned Constants
  /**
   * Time in seconds between when the robot is told to move and when the shooter actually shoots.
   */
  private static final double latency = 0.15;

  private static final Angle setpointTolerance = Degrees.of(1);
  private static final AngularVelocity maxProfiledVelocity = RotationsPerSecond.of(3);
  private static final AngularAcceleration maxProfiledAcceleration =
      RotationsPerSecondPerSecond.of(3);

  private AlignToGoal() {}

  /**
   * Create the AlignToGoal command.
   *
   * @param swerveMechanism Drivetrain to rotate toward the goal.
   * @param shooterMechanism Shooter to spin up for the current distance.
   * @param controller Driver controller; the left stick translates while aligning.
   * @param targetPose Goal pose.
   * @return {@link Command} requiring the drivetrain. The shooter runs as a forked child command, so it is only owned
   *     while this command runs and its default command resumes afterwards.
   */
  public static Command create(
      SwerveMechanism swerveMechanism,
      ShooterMechanism shooterMechanism,
      CommandNiDsXboxController controller,
      Pose2d targetPose) {
    // Maps Distance to RPM
    InterpolatingDoubleTreeMap shooterTable = new InterpolatingDoubleTreeMap();
    // Test Results
    for (var entry :
        List.of(
            Pair.of(Meters.of(1), RPM.of((1000))),
            Pair.of(Meters.of(2), RPM.of(2000)),
            Pair.of(Meters.of(3), RPM.of(3000)))) {
      shooterTable.put(entry.getFirst().in(Meters), entry.getSecond().in(RPM));
    }

    ProfiledPIDController pidController =
        new ProfiledPIDController(
            1,
            0,
            0,
            new Constraints(
                maxProfiledVelocity.in(RadiansPerSecond),
                maxProfiledAcceleration.in(RadiansPerSecondPerSecond)));
    pidController.setTolerance(setpointTolerance.in(Radians));
    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 0, 0);

    return swerveMechanism.run(coroutine -> {
      pidController.reset(swerveMechanism.getPose().getRotation().getRadians(),
                          swerveMechanism.getFieldOrientedChassisSpeed().omega);
      // Latest shot speed; the forked YAMS shooter command reads it every loop.
      LinearVelocity[] shotSpeed = {MetersPerSecond.of(0)};
      coroutine.fork(shooterMechanism.setLinearVelocity(() -> shotSpeed[0]));

      while (true) {
        // Please look here for the original authors work!
        // https://blog.eeshwark.com/robotblog/shooting-on-the-fly
        // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        // YASS did not come up with this
        // -------------------------------------------------------

        var robotSpeed = swerveMechanism.getFieldOrientedChassisSpeed();
        // 1. LATENCY COMP
        Translation2d futurePos = swerveMechanism.getPose().getTranslation().plus(
            new Translation2d(robotSpeed.vx, robotSpeed.vy).times(latency));

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
        swerveMechanism.setDriveInput(-controller.getLeftY(), -controller.getLeftX(), 0);
        var originalSpeed     = swerveMechanism.getDriveInput();
        originalSpeed.omega = output + feedforwardOutput;
        swerveMechanism.setRobotRelativeChassisSpeedsSetpoint(originalSpeed.toRobotRelative(new Rotation2d(swerveMechanism.getGyroAngle())));
        shotSpeed[0] = newHorizontalSpeed;

        coroutine.yield();
      }
    }).named("AlignToGoal");
  }
}
