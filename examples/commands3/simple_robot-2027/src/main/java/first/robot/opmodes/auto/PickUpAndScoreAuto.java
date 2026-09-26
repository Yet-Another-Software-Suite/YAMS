// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.opmodes.auto;

import static org.wpilib.units.Units.Seconds;

import first.robot.Robot;
import org.wpilib.command3.Command;
import org.wpilib.command3.Trigger;
import org.wpilib.driverstation.RobotState;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.OpMode;
import org.wpilib.units.measure.Time;

@Autonomous
public class PickUpAndScoreAuto implements OpMode {
  private static final Time PICK_UP_TIMEOUT = Seconds.of(3);

  private final Trigger enabled = new Trigger(RobotState::isEnabled);

  public PickUpAndScoreAuto(Robot robot) {
    enabled.onTrue(pickUpAndScore(robot));
  }

  /**
   * Pick up a game piece, then score it. Gives up if no game piece is seen within {@link #PICK_UP_TIMEOUT}.
   *
   * @param robot The robot to control.
   * @return {@link Command} that runs the auto.
   */
  private Command pickUpAndScore(Robot robot) {
    return Command.noRequirements(coroutine -> {
      // Race the pick up against a timeout; whichever finishes first cancels the other.
      coroutine.awaitAny(robot.arm.pickUp(), Command.waitFor(PICK_UP_TIMEOUT).named("Pick Up Timeout"));
      if (robot.arm.getBeamBreak()) {
        coroutine.await(robot.scoreHigh());
      }
    }).named("Pick Up And Score");
  }
}
