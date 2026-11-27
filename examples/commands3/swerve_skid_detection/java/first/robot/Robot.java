// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;

import first.robot.mechanisms.SwerveMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;

public class Robot extends OpModeRobot {
  public final SwerveMechanism drive = new SwerveMechanism();
  public final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(DRIVER_CONTROLLER_PORT);
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  @Override
  public void robotPeriodic() {
    drive.periodic();
    Scheduler.getDefault().run();
  }

  @Override
  public void simulationInit() {
    Notifier notifier = new Notifier(drive::simulationPeriodic);
    notifier.setName("Drive Simulation");
    notifier.startPeriodic(getPeriod());
    simulationNotifiers.add(notifier);
  }
}
