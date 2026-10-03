// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;

import org.wpilib.command2.Command;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Eject extends Command {
  /** Creates a new Intake. */

  IntakeLauncherSubsystem intakeLauncher;
  IndexerSubsystem indexer;

  public Eject(IntakeLauncherSubsystem intakeLauncher, IndexerSubsystem indexer) {
    addRequirements(intakeLauncher, indexer);
    this.intakeLauncher = intakeLauncher;
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled. Set the rollers to the
  // appropriate values for ejecting
  @Override
  public void initialize() {
    intakeLauncher.setIntakeLauncherRoller(INTAKE_EJECT_PERCENT);
    indexer.setFeederRoller(INDEXER_LAUNCHING_PERCENT);
  }

  // Called every time the scheduler runs while the command is scheduled. This
  // command doesn't require updating any values while running
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted. Stop the rollers
  @Override
  public void end(boolean interrupted) {
    intakeLauncher.setIntakeLauncherRoller(0);
    indexer.setFeederRoller(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
