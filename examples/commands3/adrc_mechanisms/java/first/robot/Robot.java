// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import first.robot.mechanisms.ADRCArmMechanism;
import first.robot.mechanisms.ADRCElevatorMechanism;
import first.robot.mechanisms.ADRCShooterMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;

/**
 * An arm, an elevator and a shooter run by ADRCs, written by FRC Team 8331. The bindings live in
 * {@link first.robot.opmodes.teleop.ADRCTeleop}, which {@link OpModeRobot} finds through its annotation. If you change
 * the name of this class or the package after creating this project, you must also update the Main.java file in the
 * project.
 */
public class Robot extends OpModeRobot {
  public final ADRCArmMechanism arm = new ADRCArmMechanism();
  public final ADRCElevatorMechanism elevator = new ADRCElevatorMechanism();
  public final ADRCShooterMechanism shooter = new ADRCShooterMechanism();

  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  /** Notifiers running each mechanism's simulation, kept so they are not garbage collected. */
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  public Robot() {
    shooter.setDefaultCommand(shooter.idle());
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, which runs the commands, among them the Live Tuning commands while they are started.
    Scheduler.getDefault().run();
    arm.periodic();
    elevator.periodic();
    shooter.periodic();
  }

  /** Runs each mechanism's simulation in its own Notifier, every robot loop period. */
  @Override
  public void simulationInit() {
    startSimulation("ADRC Arm", arm::simulationPeriodic);
    startSimulation("ADRC Elevator", elevator::simulationPeriodic);
    startSimulation("ADRC Shooter", shooter::simulationPeriodic);
  }

  private void startSimulation(String name, Runnable simulationPeriodic) {
    final Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
