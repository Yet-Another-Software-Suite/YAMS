// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import org.wpilib.command2.SequentialCommandGroup;
import first.robot.Constants.FuelConstants;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LaunchSequence extends SequentialCommandGroup {
  /** Creates a new LaunchSequence. */
  public LaunchSequence(IntakeLauncherSubsystem intakeLauncher, IndexerSubsystem indexer) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SpinUp(intakeLauncher, indexer).withTimeout(FuelConstants.SPIN_UP_SECONDS),
        new Launch(intakeLauncher, indexer));
  }
}
