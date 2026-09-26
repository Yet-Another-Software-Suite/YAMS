// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;

/**
 * Fuel handling for the 2026 FIRST KitBot. Intaking, ejecting, spinning up, and launching all run
 * the feeder and intake/launcher rollers together. Each roller is its own mechanism so it can be
 * tuned live on its own; the commands here require neither roller themselves and instead run the
 * rollers' own commands as children. Each roller is only owned while its child command runs, and
 * its idle command (which stops it) takes over again as soon as the fuel command ends. Tune the
 * voltages live with YAMS, then copy the values into {@link first.robot.Constants.FuelConstants}.
 */
public class FuelCommands {
  private final FeederMechanism feeder;
  private final IntakeLauncherMechanism intakeLauncher;

  public FuelCommands(FeederMechanism feeder, IntakeLauncherMechanism intakeLauncher) {
    this.feeder = feeder;
    this.intakeLauncher = intakeLauncher;
  }

  // Runs both rollers at their intaking voltages until interrupted.
  public Command intake() {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(feeder.intake(), intakeLauncher.intake());
    }).named("Fuel.Intake");
  }

  // Ejects fuel out the intake. Uses the same values as intaking, but in the opposite direction.
  // Runs until interrupted.
  public Command eject() {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(feeder.eject(), intakeLauncher.eject());
    }).named("Fuel.Eject");
  }

  // Spins up for SPIN_UP_SECONDS, then launches until interrupted.
  public Command spinUpAndLaunch() {
    return Command.noRequirements(coroutine -> {
      // The launcher runs at launching voltage the whole time; only the feeder changes. Forked
      // commands live until they are interrupted or this command ends.
      coroutine.fork(intakeLauncher.launch(), feeder.spinUp());
      coroutine.wait(Seconds.of(SPIN_UP_SECONDS));
      // Scheduling the feeder's launch command interrupts its spin-up command, since both are
      // children of this command and require the feeder
      coroutine.await(feeder.launch());
    }).named("Fuel.SpinUpAndLaunch");
  }
}
