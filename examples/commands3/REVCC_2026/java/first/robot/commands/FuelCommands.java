// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import first.robot.mechanisms.ConveyorMechanism;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeMechanism;
import first.robot.mechanisms.ShooterMechanism;
import org.wpilib.command3.Command;

/**
 * Commands that run more than one fuel mechanism together. Each mechanism is its own
 * {@link org.wpilib.command3.Mechanism} with its own commands, so it can be tuned live on its own.
 * These coroutines have no requirements of their own: they await the mechanism commands, so each
 * mechanism is only owned while its command runs. When a mechanism command ends or is canceled, that
 * mechanism's default command stops it.
 */
public final class FuelCommands
{
  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public static Command intake(IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(intake.intake(), conveyor.intake());
    }).named("Intaking");
  }

  /**
   * Command to reverse the intake motor and conveyor motors. When the command is interrupted, e.g.
   * the button is released, the motors will stop.
   */
  public static Command extake(IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(intake.extake(), conveyor.extake());
    }).named("Extaking");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the feeder stops and the flywheel coasts down.
   */
  public static Command feed(ShooterMechanism shooter, FeederMechanism feeder)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(shooter.spinUp(), feeder.feed());
    }).named("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches the
   * desired speed it starts the Feeder. When the command is interrupted the feeder stops and the
   * flywheel coasts down.
   */
  public static Command shoot(ShooterMechanism shooter, FeederMechanism feeder)
  {
    return Command.noRequirements(coroutine -> {
      // The flywheel keeps spinning in the background while this command waits for it.
      coroutine.fork(shooter.spinUp());
      coroutine.waitUntil(shooter.isFlywheelSpinning);
      // The feeder is only claimed once the flywheel is at speed.
      coroutine.await(feeder.feed());
    }).named("Shooting");
  }

  /** Runs {@link #shoot} and {@link #intake} together until canceled. */
  public static Command shootAndIntake(ShooterMechanism shooter, FeederMechanism feeder,
                                       IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(shoot(shooter, feeder), intake(intake, conveyor));
    }).named("Shooting and Intaking");
  }

  private FuelCommands()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
