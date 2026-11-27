// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first;

import first.robot.Robot;
import org.wpilib.framework.RobotBase;

/** Do not add any static variables to this class, or any initialization at all. */
public final class Main {
  private Main() {}

  public static void main(String... args) {
    RobotBase.startRobot(Robot::new);
  }
}
