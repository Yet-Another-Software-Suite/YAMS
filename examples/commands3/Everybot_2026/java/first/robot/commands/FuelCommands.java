// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.FuelConstants;
import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;

/**
 * Fuel commands that drive the intake/launcher and the indexer together. These replace the
 * Everybot's Intake, Eject, SpinUp, Launch and LaunchSequence command classes. Each one requires
 * both mechanisms for its whole run, like the original commands did, and runs until interrupted.
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
    return Command.requiring(intakeLauncher, indexer)
        .executing(coroutine -> coroutine.awaitAll(intakeLauncher.intake(), indexer.intake()))
        .named("Intake");
  }

  /**
   * Ejects fuel back out through the intake.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the eject command
   */
  public static Command eject(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.requiring(intakeLauncher, indexer)
        .executing(coroutine -> coroutine.awaitAll(intakeLauncher.eject(), indexer.feed()))
        .named("Eject");
  }

  /**
   * Spins up the launcher while the indexer holds the fuel back.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the spin-up command
   */
  public static Command spinUp(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.requiring(intakeLauncher, indexer)
        .executing(coroutine -> coroutine.awaitAll(intakeLauncher.launch(), indexer.spinUp()))
        .named("SpinUp");
  }

  /**
   * Launches fuel: the launcher spins and the indexer feeds it.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the launch command
   */
  public static Command launch(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.requiring(intakeLauncher, indexer)
        .executing(coroutine -> coroutine.awaitAll(intakeLauncher.launch(), indexer.feed()))
        .named("Launch");
  }

  /**
   * Spins up for {@link FuelConstants#SPIN_UP_SECONDS}, then launches until interrupted.
   *
   * @param intakeLauncher the intake/launcher rollers
   * @param indexer        the indexer
   * @return the launch sequence command
   */
  public static Command launchSequence(IntakeLauncherMechanism intakeLauncher, IndexerMechanism indexer) {
    return Command.requiring(intakeLauncher, indexer)
        .executing(coroutine -> {
          coroutine.await(spinUp(intakeLauncher, indexer).withTimeout(Seconds.of(FuelConstants.SPIN_UP_SECONDS)));
          coroutine.await(launch(intakeLauncher, indexer));
        })
        .named("LaunchSequence");
  }
}
