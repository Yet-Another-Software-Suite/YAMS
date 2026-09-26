// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;
import first.robot.subsystems.CANDriveSubsystem;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ExampleAuto extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto(CANDriveSubsystem driveSubsystem, IntakeLauncherSubsystem intakeLauncher, IndexerSubsystem indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    // Drive backwards for 3 seconds. The driveArcadeAuto command factory
    // intentionally creates a command which does not end which allows us to control
    // the timing using the withTimeout decorator
    new AutoDrive(driveSubsystem,0.5,  0.0).withTimeout(3),
    // Spin up the launcher for 0.75 second and then launch balls for 9.25 seconds, for a
    // total of 10 seconds
    new LaunchSequence(intakeLauncher, indexer).withTimeout(10));


  }
}
