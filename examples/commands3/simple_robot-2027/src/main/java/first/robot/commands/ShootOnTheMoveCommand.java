// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Radians;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import first.robot.mechanisms.HoodMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.TurretMechanism;
import java.util.function.Supplier;
import yams.commands3.swerve.SwerveDrive;

/**
 * Adapted from 6328 Mechanical Advantage! Original source is here:
 * https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java
 */
public final class ShootOnTheMoveCommand {
  private static final InterpolatingTreeMap<Double, Rotation2d> launchHoodAngleMap     =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap launchFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Tuning Constants
  private static final double phaseDelay = 0.05;
  private static final Distance minDistance = Feet.of(1);
  private static final Distance maxDistance = Meters.of(5);

  static {
    // These should be found on your robot
    launchHoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    launchHoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    launchHoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    launchHoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    launchHoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    launchHoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    launchHoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    launchHoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    launchHoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    launchFlywheelSpeedMap.put(1.34, 2100.0);
    launchFlywheelSpeedMap.put(1.78, 2200.0);
    launchFlywheelSpeedMap.put(2.17, 2200.0);
    launchFlywheelSpeedMap.put(2.81, 2300.0);
    launchFlywheelSpeedMap.put(3.82, 2500.0);
    launchFlywheelSpeedMap.put(4.09, 2550.0);
    launchFlywheelSpeedMap.put(4.40, 2600.0);
    launchFlywheelSpeedMap.put(4.77, 2650.0);
    launchFlywheelSpeedMap.put(5.57, 2750.0);
    launchFlywheelSpeedMap.put(5.60, 2900.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
  }

  private ShootOnTheMoveCommand() {}

  /** One shot solution: where to point the turret and hood, and how fast to spin the flywheel. */
  private record Shot(Angle turretAngle, Angle hoodAngle, AngularVelocity flywheelSpeed) {}

  /**
   * Create the ShootOnTheMove command.
   *
   * @param turret Turret to aim at the hub.
   * @param shooterMechanism Shooter to spin up for the lookahead distance.
   * @param hoodMechanism Hood to angle for the lookahead distance.
   * @param swerveDrive Drivetrain used for the robot pose and velocity.
   * @return {@link Command} with no requirements of its own. Once the first in-range shot is found it forks one YAMS
   *     command each for the turret, hood and shooter, which follow the latest in-range shot.
   */
  public static Command create(
      TurretMechanism turret,
      ShooterMechanism shooterMechanism,
      HoodMechanism hoodMechanism,
      SwerveDrive swerveDrive) {
    Supplier<Pose2d> estimatedPose = () -> {
      // Calculate estimated pose while accounting for phase delay
      ChassisVelocities robotRelativeVelocity = swerveDrive.getRobotRelativeSpeed();
      var           robotPose             = swerveDrive.getPose();

      robotPose = robotPose.transformBy(
          robotRelativeVelocity.toTwist2d(phaseDelay).exp());
      // Optional, add logging here
      swerveDrive.getField2d().getObject("ShootOnTheMovePose").setPose(robotPose);
      return robotPose;
    };
    Supplier<ChassisVelocities> fieldRelativeVelocitySupplier = swerveDrive::getFieldRelativeSpeed;

    return Command.noRequirements(coroutine -> {
      // Latest in-range shot, null until the first one is found.
      Shot[] shot = {null};
      Debouncer shootingDebounce = new Debouncer(0.1, DebounceType.FALLING);

      // Command-scoped trigger: this binding only exists while ShootOnTheMove runs.
      new Trigger(() -> shot[0] != null
                        && shootingDebounce.calculate(
                            shooterMechanism.getVelocity().isNear(shot[0].flywheelSpeed(), RPM.of(10))))
          .whileTrue(Command.noRequirements(feed -> {
            // Set indexer to go vrooooom
            // HERE, e.g. feed.await(indexer.feed());
            feed.park();
          }).named("ShootOnTheMove Feed"));

      boolean aiming = false;
      while (true) {
        // Get estimated pose
        var robotPose = estimatedPose.get();
        var fieldRelativeVelocity = fieldRelativeVelocitySupplier.get();

        // Calculate distance from turret to target
        Translation2d target =
            AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());
        Pose2d turretPosition = turret.getPose(robotPose);
        double turretToTargetDistance = target.getDistance(turretPosition.getTranslation());

        // Calculate field relative turret velocity
        Angle         robotAngle     = robotPose.getRotation().getMeasure();
        ChassisVelocities turretVelocity = turret.getVelocity(fieldRelativeVelocity, robotAngle);

        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Pose2d lookaheadPose = turretPosition;
        double lookaheadTurretToTargetDistance = turretToTargetDistance;
        for (int i = 0; i < 20; i++) {
          timeOfFlight = timeOfFlightMap.get(lookaheadTurretToTargetDistance);
          double offsetX = turretVelocity.vx * timeOfFlight;
          double offsetY = turretVelocity.vy * timeOfFlight;
          lookaheadPose =
              new Pose2d(
                  turretPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
                  turretPosition.getRotation());
          lookaheadTurretToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
        }

        // Calculate parameters accounted for imparted velocity
        Rotation2d turretAngle =
            target.minus(lookaheadPose.getTranslation()).getAngle().orElse(Rotation2d.ZERO);
        double hoodAngle = launchHoodAngleMap.get(lookaheadTurretToTargetDistance).getRadians();
        var lookaheadTurretToTargetDistanceMeasure = Meters.of(lookaheadTurretToTargetDistance);
        if (lookaheadTurretToTargetDistanceMeasure.gte(minDistance)
            && lookaheadTurretToTargetDistanceMeasure.lte(maxDistance)) {
          var shooterRPM = RPM.of(launchFlywheelSpeedMap.get(lookaheadTurretToTargetDistance));
          shot[0] = new Shot(turretAngle.getMeasure(), Radians.of(hoodAngle), shooterRPM);
        }

        if (!aiming && shot[0] != null) {
          // Each mechanism is only owned by its forked command, which ends with this command.
          coroutine.fork(
              turret.setAngle(() -> shot[0].turretAngle()),
              hoodMechanism.setAngle(() -> shot[0].hoodAngle()),
              shooterMechanism.setVelocity(() -> shot[0].flywheelSpeed()));
          aiming = true;
        }

        coroutine.yield();
      }
    }).whenCanceled(() -> shooterMechanism.setDutyCycleSetpoint(0)).named("ShootOnTheMove");
  }
}
