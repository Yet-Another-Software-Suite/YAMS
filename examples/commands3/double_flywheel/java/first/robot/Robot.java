// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.*;

import first.robot.mechanisms.DoubleFlyWheelMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;

/**
 * The robot: a double flywheel and the operator's controller. The bindings live in
 * {@link first.robot.opmodes.teleop.DoubleFlyWheelTeleop}, which {@link OpModeRobot} finds through its annotation. If
 * you change the name of this class or the package after creating this project, you must also update the Main.java
 * file in the project.
 */
public class Robot extends OpModeRobot {
  public final DoubleFlyWheelMechanism doubleFlyWheel = new DoubleFlyWheelMechanism();

  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(OPERATOR_CONTROLLER_PORT);

  /** Runs the double flywheel's simulation, kept so it is not garbage collected. */
  private Notifier simulationNotifier;

  public Robot() {
    doubleFlyWheel.setDefaultCommand(doubleFlyWheel.idle());
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, which runs the commands, among them the Live Tuning command while it is started.
    Scheduler.getDefault().run();
    doubleFlyWheel.periodic();
  }

  /** Runs the double flywheel's simulation in its own Notifier, every robot loop period. */
  @Override
  public void simulationInit() {
    simulationNotifier = new Notifier(doubleFlyWheel::simulationPeriodic);
    simulationNotifier.setName("Double FlyWheel Simulation");
    simulationNotifier.startPeriodic(getPeriod());
  }
}
