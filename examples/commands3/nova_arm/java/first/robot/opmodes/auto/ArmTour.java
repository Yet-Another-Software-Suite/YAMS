// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import static first.robot.Constants.ArmConstants.*;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.units.measure.Angle;

/** Swing through every setpoint, waiting at each one, then stow. Watch the arm move without a controller. */
@Autonomous(name = "Arm Tour")
public class ArmTour implements OpMode {
  /**
   * Creates the autonomous opmode. The OpModeRobot framework calls this when the opmode is selected on the driver
   * station.
   *
   * @param robot The robot instance to control.
   */
  public ArmTour(Robot robot) {
    // The tour starts when autonomous is enabled and is canceled when it is disabled.
    RobotModeTriggers.autonomous().whileTrue(tour(robot));
  }

  /**
   * The tour itself, also run by the tests.
   *
   * @param robot The robot.
   * @return {@link Command} that ends once the arm is stowed.
   */
  public static Command tour(Robot robot) {
    return Command.noRequirements(coroutine -> {
      for (Angle angle : new Angle[] {LEVEL, HIGH, UP, LEVEL, STOW}) {
        // Hold the angle until the arm has settled there; ending settleAt cancels the hold.
        coroutine.awaitAny(robot.arm.setAngle(angle), robot.arm.settleAt(angle));
      }
    }).named("Arm Tour");
  }
}
