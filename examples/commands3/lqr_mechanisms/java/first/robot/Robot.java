// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import first.robot.mechanisms.LQRArmMechanism;
import first.robot.mechanisms.LQRFlyWheelMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;

/**
 * An arm and a flywheel run by LQRs. The bindings live in {@link first.robot.opmodes.teleop.LQRTeleop}, which
 * {@link OpModeRobot} finds through its annotation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends OpModeRobot {
  public final LQRArmMechanism arm = new LQRArmMechanism();
  public final LQRFlyWheelMechanism flyWheel = new LQRFlyWheelMechanism();

  public final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  /** Notifiers running each mechanism's simulation, kept so they are not garbage collected. */
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  public Robot() {
    flyWheel.setDefaultCommand(flyWheel.idle());
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, which runs the commands, among them the Live Tuning commands while they are started.
    Scheduler.getDefault().run();
    arm.periodic();
    flyWheel.periodic();
  }

  /** Runs each mechanism's simulation in its own Notifier, every robot loop period. */
  @Override
  public void simulationInit() {
    startSimulation("LQR Arm", arm::simulationPeriodic);
    startSimulation("LQR FlyWheel", flyWheel::simulationPeriodic);
  }

  private void startSimulation(String name, Runnable simulationPeriodic) {
    final Notifier notifier = new Notifier(simulationPeriodic);
    notifier.setName(name + " Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
