// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.NeedsNameBuilderStage;
import org.wpilib.units.measure.Voltage;

/**
 * Fuel handling for the 2026 FIRST KitBot. Intaking, ejecting, spinning up, and launching all run
 * the feeder and intake/launcher rollers together. Each roller is its own mechanism so it can be
 * tuned live on its own; the commands here require both and run one roller command on each. Tune
 * the voltages live with YAMS, then copy the values into
 * {@link first.robot.Constants.FuelConstants}.
 */
public class FuelCommands {
  private final FeederMechanism feeder;
  private final IntakeLauncherMechanism intakeLauncher;

  public FuelCommands(FeederMechanism feeder, IntakeLauncherMechanism intakeLauncher) {
    this.feeder = feeder;
    this.intakeLauncher = intakeLauncher;
  }

  // A method to stop the rollers
  public void stop() {
    feeder.stop();
    intakeLauncher.stop();
  }

  // Starts building a command that runs both rollers at the given voltages until interrupted. The
  // rollers keep their last output when it ends unless the caller adds a whenCanceled stop.
  private NeedsNameBuilderStage runRollers(Voltage feederVoltage, Voltage intakeLauncherVoltage) {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> coroutine.awaitAll(
            feeder.runAt(feederVoltage),
            intakeLauncher.runAt(intakeLauncherVoltage)));
  }

  // Sets the rollers to values for intaking. Stops the rollers when interrupted.
  public Command intake() {
    return runRollers(INTAKING_FEEDER_VOLTAGE, INTAKING_INTAKE_VOLTAGE)
        .whenCanceled(this::stop)
        .named("Fuel.Intake");
  }

  // Sets the rollers to values for ejecting fuel out the intake. Uses the same values as intaking,
  // but in the opposite direction. Stops the rollers when interrupted.
  public Command eject() {
    return runRollers(INTAKING_FEEDER_VOLTAGE.unaryMinus(), INTAKING_INTAKE_VOLTAGE.unaryMinus())
        .whenCanceled(this::stop)
        .named("Fuel.Eject");
  }

  // Spins up the launcher roller while spinning the feeder roller to push Fuel away from the
  // launcher. Runs until interrupted and leaves the rollers running.
  public Command spinUp() {
    return runRollers(SPIN_UP_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE).named("Fuel.SpinUp");
  }

  // Sets the rollers to values for launching. Runs until interrupted and leaves the rollers
  // running.
  public Command launch() {
    return runRollers(LAUNCHING_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE).named("Fuel.Launch");
  }

  // Spins up for SPIN_UP_SECONDS, then launches until interrupted. Stops the rollers when
  // interrupted.
  public Command spinUpAndLaunch() {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> {
          coroutine.await(spinUp().withTimeout(Seconds.of(SPIN_UP_SECONDS)));
          coroutine.await(launch());
        })
        .whenCanceled(this::stop)
        .named("Fuel.SpinUpAndLaunch");
  }

  // Stops the rollers once and ends immediately.
  public Command stopCommand() {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> stop())
        .named("Fuel.Stop");
  }
}
