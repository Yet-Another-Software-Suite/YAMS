// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.OperatorConstants.DRIVER_CONTROLLER_PORT;
import static first.robot.Constants.PowerHealthConstants.BATTERY_CAPACITY_AH;
import static first.robot.Constants.PowerHealthConstants.SIM_BATTERY_FULL_VOLTAGE;
import static first.robot.Constants.PowerHealthConstants.SIM_BATTERY_INTERNAL_RESISTANCE;

import first.robot.mechanisms.SwerveMechanism;
import java.util.ArrayList;
import java.util.List;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.system.Notifier;
import yams.core.motorcontrollers.simulation.BatterySim;

public class Robot extends OpModeRobot {
  public final SwerveMechanism drive;
  public final CommandNiDsXboxController driverController =
      new CommandNiDsXboxController(DRIVER_CONTROLLER_PORT);
  private final List<Notifier> simulationNotifiers = new ArrayList<>();

  public Robot() {
    BatterySim.enableDischarge(
        BATTERY_CAPACITY_AH, SIM_BATTERY_FULL_VOLTAGE, SIM_BATTERY_INTERNAL_RESISTANCE);
    drive = new SwerveMechanism();
  }

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
