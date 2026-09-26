// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;

/**
 * Fuel commands that run the intake/launcher and the indexer together. These replace the
 * Everybot's Intake, Eject and LaunchSequence command classes; the SpinUp and Launch steps are
 * steps of the launch sequence.
 *
 * <p>
 * Each one is a coroutine with no requirements of its own that runs the mechanisms' own commands.
 * A mechanism is only owned while its command runs, and each mechanism command stops its roller
 * when it is canceled. All of them run until interrupted.
 */
public final class FuelCommands {
  private FuelCommands() {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Intakes fuel off the ground.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the intake command
   */
  public static Command intake(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(intakeLauncher.intake(), indexer.intake());
    }).named("Intake");
  }

  /**
   * Ejects fuel back out through the intake.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the eject command
   */
  public static Command eject(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(intakeLauncher.eject(), indexer.feed());
    }).named("Eject");
  }

  /**
   * Spins up the launcher for {@link first.robot.Constants.FuelConstants#SPIN_UP_SECONDS} while the
   * indexer holds the fuel back, then feeds fuel into the launcher until interrupted.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the launch sequence command
   */
  public static Command launchSequence(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.noRequirements(coroutine -> {
      // Spin up. The launcher keeps running for the rest of the sequence.
      coroutine.fork(intakeLauncher.launch(), indexer.holdBack());
      coroutine.wait(Seconds.of(SPIN_UP_SECONDS));
      // Launch. Feeding interrupts its sibling holdBack() on the indexer.
      coroutine.await(indexer.feed());
    }).named("LaunchSequence");
  }
}
