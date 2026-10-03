// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import first.robot.Robot;
import first.robot.commands.Autos;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

/** The REV example auto: drives an 's' curve that ends 3 meters ahead of the start. */
@Autonomous
public class ExampleAuto implements OpMode
{
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  /**
   * Creates the autonomous opmode. This constructor is called automatically by the OpModeRobot
   * framework when the opmode is selected.
   *
   * @param robot The robot instance to control.
   */
  public ExampleAuto(Robot robot)
  {
    // Start the routine once the robot is enabled. It is canceled when the opmode exits.
    enabled.onTrue(Autos.exampleAuto(robot.drive));
  }
}
