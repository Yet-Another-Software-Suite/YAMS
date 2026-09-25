// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeLauncherMechanism;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.Voltage;

/**
 * Fuel handling for the 2026 FIRST KitBot. Intaking, ejecting, spinning up, and launching all run
 * the feeder and intake/launcher rollers together. Each roller is its own mechanism so it can be
 * tuned live on its own; the commands here require both and set each roller directly. Tune the
 * voltages live with YAMS, then copy the values into {@link first.robot.Constants.FuelConstants}.
 */
public class FuelCommands {
  private final FeederMechanism feeder;
  private final IntakeLauncherMechanism intakeLauncher;

  public FuelCommands(FeederMechanism feeder, IntakeLauncherMechanism intakeLauncher) {
    this.feeder = feeder;
    this.intakeLauncher = intakeLauncher;
  }

  // Sets both rollers to the given voltages. They hold that output until set again.
  public void setRollers(Voltage feederVoltage, Voltage intakeLauncherVoltage) {
    feeder.setVoltage(feederVoltage);
    intakeLauncher.setVoltage(intakeLauncherVoltage);
  }

  // Sets the rollers to values for spinning up the launcher while pushing Fuel away from it.
  public void setSpinUp() {
    setRollers(SPIN_UP_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // Sets the rollers to values for launching.
  public void setLaunch() {
    setRollers(LAUNCHING_FEEDER_VOLTAGE, LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // A method to stop the rollers
  public void stop() {
    feeder.stop();
    intakeLauncher.stop();
  }

  // Sets the rollers to values for intaking. Stops the rollers when interrupted.
  public Command intake() {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> {
          setRollers(INTAKING_FEEDER_VOLTAGE, INTAKING_INTAKE_VOLTAGE);
          coroutine.park();
        })
        .whenCanceled(this::stop)
        .named("Fuel.Intake");
  }

  // Sets the rollers to values for ejecting fuel out the intake. Uses the same values as intaking,
  // but in the opposite direction. Stops the rollers when interrupted.
  public Command eject() {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> {
          setRollers(INTAKING_FEEDER_VOLTAGE.unaryMinus(), INTAKING_INTAKE_VOLTAGE.unaryMinus());
          coroutine.park();
        })
        .whenCanceled(this::stop)
        .named("Fuel.Eject");
  }

  // Spins up for SPIN_UP_SECONDS, then launches until interrupted. Stops the rollers when
  // interrupted.
  public Command spinUpAndLaunch() {
    return Command.requiring(feeder, intakeLauncher)
        .executing(coroutine -> {
          setSpinUp();
          coroutine.wait(Seconds.of(SPIN_UP_SECONDS));
          setLaunch();
          coroutine.park();
        })
        .whenCanceled(this::stop)
        .named("Fuel.SpinUpAndLaunch");
  }
}
