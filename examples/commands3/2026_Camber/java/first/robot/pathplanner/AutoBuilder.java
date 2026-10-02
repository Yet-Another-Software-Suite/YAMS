// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Stand-in for PathPlannerLib's {@code AutoBuilder}, which is built on commands v2 and cannot be
 * used alongside commands v3. Commands v3 support for PathPlanner is not finished yet, so every auto
 * reports an error and does nothing. Teleop is unaffected. The autos in
 * {@code deploy/pathplanner/autos} and the named commands registered in {@code Robot} are kept for
 * when it is.
 */
public final class AutoBuilder
{

  private AutoBuilder()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Build the command for an auto. Not available yet: reports an error and does nothing.
   *
   * @param autoName Name of the auto in the PathPlanner GUI.
   * @return A command that does nothing.
   */
  public static Command buildAuto(String autoName)
  {
    DriverStationErrors.reportError("PathPlanner is not available with commands v3 yet; auto " + autoName +
                                    " does nothing", false);
    return Command.noRequirements(coroutine -> {}).named(autoName + " (PathPlanner unavailable)");
  }
}
