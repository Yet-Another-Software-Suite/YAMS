// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.mechanisms.ArmMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;

/**
 * An arm driven by a FIRST A301. The bindings live in {@link first.robot.opmodes.teleop.ArmTeleop}, and the
 * autonomous in {@link first.robot.opmodes.auto.ArmTour}, which {@link OpModeRobot} finds through their annotations.
 * If you change the name of this class or the package after creating this project, you must also update the Main.java
 * file in the project.
 */
public class Robot extends OpModeRobot {
  public final ArmMechanism arm = new ArmMechanism();

  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(OPERATOR_CONTROLLER_PORT);

  /** Notifier running the arm's simulation, kept so it is not garbage collected. */
  private Notifier simulationNotifier;

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, which runs the commands.
    Scheduler.getDefault().run();
    arm.periodic();
  }

  /** Runs the arm's simulation in its own Notifier, every robot loop period. */
  @Override
  public void simulationInit() {
    simulationNotifier = new Notifier(arm::simulationPeriodic);
    simulationNotifier.setName("Arm Simulation");
    simulationNotifier.startPeriodic(getPeriod());
  }
}
