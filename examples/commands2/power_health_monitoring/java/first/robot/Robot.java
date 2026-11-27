// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.PowerHealthConstants.BATTERY_CAPACITY_AH;
import static first.robot.Constants.PowerHealthConstants.SIM_BATTERY_FULL_VOLTAGE;
import static first.robot.Constants.PowerHealthConstants.SIM_BATTERY_INTERNAL_RESISTANCE;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;
import yams.core.motorcontrollers.simulation.BatterySim;

public class Robot extends TimedRobot {
  public final RobotContainer robotContainer;

  public Robot() {
    BatterySim.enableDischarge(
        BATTERY_CAPACITY_AH, SIM_BATTERY_FULL_VOLTAGE, SIM_BATTERY_INTERNAL_RESISTANCE);
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
}
