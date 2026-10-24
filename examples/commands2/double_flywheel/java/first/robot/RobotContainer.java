// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static first.robot.Constants.DoubleFlyWheelConstants.*;
import static first.robot.Constants.OperatorConstants.*;

import first.robot.subsystems.DoubleFlyWheelSubsystem;
import org.wpilib.command2.button.CommandNiDsXboxController;

/** Holds the double flywheel and binds it to the operator's controller. */
public class RobotContainer {
  public final DoubleFlyWheelSubsystem doubleFlyWheel = new DoubleFlyWheelSubsystem();

  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(OPERATOR_CONTROLLER_PORT);

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    // Hold the right bumper to spin both wheels up for a shot.
    operatorController.rightBumper().whileTrue(doubleFlyWheel.spinUp(SHOT_LOWER_SPEED, SHOT_UPPER_SPEED));
    // Press Start to live tune both wheels from code, the same command the YAMS Live Tuner starts. Press it again to
    // stop.
    operatorController.start().toggleOnTrue(doubleFlyWheel.liveTune());
  }
}
