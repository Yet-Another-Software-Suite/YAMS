// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Notifier;

/**
 * The robot: a swerve drive, a shooter and a feeder. The bindings live in
 * {@link first.robot.opmodes.teleop.ShootOnTheMoveTeleop}, which {@link OpModeRobot} finds through its annotation. If
 * you change the name of this class or the package after creating this project, you must also update the Main.java
 * file in the project.
 */
public class Robot extends OpModeRobot {
  public final SwerveMechanism drive = new SwerveMechanism();
  public final ShooterMechanism shooter = new ShooterMechanism();
  public final FeederMechanism feeder = new FeederMechanism();

  public final CommandNiDsXboxController driverController = new CommandNiDsXboxController(DRIVER_CONTROLLER_PORT);

  /** Each mechanism's simulation Notifier, kept so that it is not garbage collected. */
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  /**
   * The shot for where the robot is, and how it is moving, right now. To try another strategy, change this one line:
   * {@code ShotCalculator.newton(...)} with the same arguments, or {@code ShotCalculator.standing(drive.getPose())}.
   */
  public ShotCalculator.Shot shot() {
    return ShotCalculator.recursive(drive.getPose(), drive.getFieldRelativeSpeed());
  }

  public Robot() {
    // With no button held, the flywheel coasts and the feeder is stopped.
    shooter.setDefaultCommand(shooter.idle());
    feeder.setDefaultCommand(feeder.idle());
  }

  @Override
  public void robotPeriodic() {
    // Update every mechanism before the scheduler runs its commands.
    drive.periodic();
    shooter.periodic();
    feeder.periodic();
    Scheduler.getDefault().run();
    // Draw the real target and the aim point on the drive's field, to watch the lead in simulation.
    drive.getField2d().getObject("Target").setPose(new Pose2d(ShotCalculator.TARGET, Rotation2d.ZERO));
    drive.getField2d().getObject("Aim point").setPose(shot().target());
  }

  @Override
  public void simulationInit() {
    // Run each mechanism's simulation in its own Notifier, every robot loop.
    startSimulation("Drive", drive::simulationPeriodic);
    startSimulation("Shooter", shooter::simulationPeriodic);
    startSimulation("Feeder", feeder::simulationPeriodic);
  }

  /** Runs a mechanism's simulation in its own Notifier, every robot loop period. */
  private void startSimulation(String name, Runnable simulationPeriodic) {
    Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
