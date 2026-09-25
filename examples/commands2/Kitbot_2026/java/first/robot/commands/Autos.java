// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.subsystems.CANDriveSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.SequentialCommandGroup;

public final class Autos {
  // Example autonomous command which drives forward for 1 second.
  public static final Command exampleAuto(CANDriveSubsystem driveSubsystem, FuelCommands fuel) {
    return new SequentialCommandGroup(
        // Drive backwards for .25 seconds. The driveArcadeAuto command factory
        // creates a command which does not end which allows us to control
        // the timing using the withTimeout decorator
        driveSubsystem.driveArcade(() -> 0.5, () -> 0).withTimeout(.25),
        // Stop driving. This line uses the regular driveArcade command factory so it
        // ends immediately after commanding the motors to stop
        driveSubsystem.driveArcade(() -> 0, () -> 0),
        // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
        // total of 10 seconds
        fuel.spinUpCommand().withTimeout(1),
        fuel.launchCommand().withTimeout(9),
        // Stop running the launcher
        fuel.stopCommand());
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
