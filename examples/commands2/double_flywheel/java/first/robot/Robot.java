// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import org.wpilib.command2.CommandScheduler;
import org.wpilib.framework.TimedRobot;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public final RobotContainer robotContainer;

  public Robot() {
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, and with it each subsystem's periodic(), which publishes the YAMS telemetry.
    CommandScheduler.getInstance().run();
  }
}
