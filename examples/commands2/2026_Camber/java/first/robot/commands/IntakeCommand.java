// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import org.wpilib.command2.Command;
import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.ShooterSubsystem;


public class IntakeCommand extends Command
{

  private final IndexerSubsystem indexerSubsystem;
  private final ShooterSubsystem shooterSubsystem;

  public IntakeCommand(IndexerSubsystem indexerSubsystem, ShooterSubsystem shooterSubsystem)
  {
    this.indexerSubsystem = indexerSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    // each subsystem used by the command must be passed into the
    // addRequirements() method (which takes a vararg of Subsystem)
    addRequirements(this.indexerSubsystem, this.shooterSubsystem);
  }

  @Override
  public void initialize()
  {

  }

  @Override
  public void execute()
  {
    shooterSubsystem.setDutycycle(0.4);
    indexerSubsystem.setDutycycle(0.8);
  }

  @Override
  public boolean isFinished()
  {
    // TODO: Make this return true when this Command no longer needs to run execute()
    return false;
  }

  @Override
  public void end(boolean interrupted)
  {
    shooterSubsystem.setDutycycle(0);
    indexerSubsystem.setDutycycle(0);
  }
}
