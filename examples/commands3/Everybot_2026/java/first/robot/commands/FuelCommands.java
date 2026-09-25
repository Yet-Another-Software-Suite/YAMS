// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;

/**
 * Fuel commands that drive the intake/launcher and the indexer together. These replace the
 * Everybot's Intake, Eject and LaunchSequence command classes; the SpinUp and Launch steps are
 * written inline in the launch sequence. Each one requires both mechanisms for its whole run, like
 * the original commands did, runs until interrupted, and stops both rollers when canceled.
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
    return Command.requiring(intakeLauncher, indexer).executing(coroutine -> {
      intakeLauncher.setPower(INTAKE_INTAKING_PERCENT);
      indexer.setPower(INDEXER_INTAKING_PERCENT);
      coroutine.park();
    }).whenCanceled(() -> {
      intakeLauncher.stop();
      indexer.stop();
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
    return Command.requiring(intakeLauncher, indexer).executing(coroutine -> {
      intakeLauncher.setPower(INTAKE_EJECT_PERCENT);
      indexer.setPower(INDEXER_LAUNCHING_PERCENT);
      coroutine.park();
    }).whenCanceled(() -> {
      intakeLauncher.stop();
      indexer.stop();
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
    return Command.requiring(intakeLauncher, indexer).executing(coroutine -> {
      // Spin up
      intakeLauncher.setPower(LAUNCHING_LAUNCHER_PERCENT);
      indexer.setPower(INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT);
      coroutine.wait(Seconds.of(SPIN_UP_SECONDS));
      // Launch
      indexer.setPower(INDEXER_LAUNCHING_PERCENT);
      coroutine.park();
    }).whenCanceled(() -> {
      intakeLauncher.stop();
      indexer.stop();
    }).named("LaunchSequence");
  }
}
