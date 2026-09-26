// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;

@Autonomous
public class NoAuto implements OpMode {
  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  public NoAuto(Robot robot) {
    enabled.onTrue(
        Command.noRequirements(coroutine -> System.out.println("No autonomous command configured"))
            .named("No Auto"));
  }
}
