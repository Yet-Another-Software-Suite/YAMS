// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;

import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

/**
 * Fuel handling for the 2026 FIRST KitBot. Intaking, ejecting, spinning up, and launching all run
 * the feeder and intake/launcher rollers together. Each roller is its own subsystem so it can be
 * tuned live on its own; the commands here require both. Tune the voltages live with YAMS,
 * then copy the values into {@link first.robot.Constants.FuelConstants}.
 */
public class FuelCommands {
  private final FeederSubsystem feeder;
  private final IntakeLauncherSubsystem intakeLauncher;

  public FuelCommands(FeederSubsystem feeder, IntakeLauncherSubsystem intakeLauncher) {
    this.feeder = feeder;
    this.intakeLauncher = intakeLauncher;
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feeder.setVoltage(INTAKING_FEEDER_VOLTAGE);
    intakeLauncher.setVoltage(INTAKING_INTAKE_VOLTAGE);
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feeder.setVoltage(INTAKING_FEEDER_VOLTAGE.unaryMinus());
    intakeLauncher.setVoltage(INTAKING_INTAKE_VOLTAGE.unaryMinus());
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feeder.setVoltage(LAUNCHING_FEEDER_VOLTAGE);
    intakeLauncher.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // A method to stop the rollers
  public void stop() {
    feeder.stop();
    intakeLauncher.stop();
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feeder.setVoltage(SPIN_UP_FEEDER_VOLTAGE);
    intakeLauncher.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE);
  }

  // Command factories; each requires both rollers.
  public Command intakeCommand() {
    return Commands.runEnd(this::intake, this::stop, feeder, intakeLauncher);
  }

  public Command ejectCommand() {
    return Commands.runEnd(this::eject, this::stop, feeder, intakeLauncher);
  }

  public Command spinUpCommand() {
    return Commands.run(this::spinUp, feeder, intakeLauncher);
  }

  public Command launchCommand() {
    return Commands.run(this::launch, feeder, intakeLauncher);
  }

  public Command stopCommand() {
    return Commands.runOnce(this::stop, feeder, intakeLauncher);
  }
}
