// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static first.robot.Constants.FuelConstants.*;
import static org.wpilib.units.Units.Volts;

import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.IntakeLauncherSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.tunable.TunableDouble;
import org.wpilib.tunable.Tunables;

/**
 * Fuel handling for the 2026 FIRST KitBot. Intaking, ejecting, spinning up, and launching all run
 * the feeder and intake/launcher rollers together. Each roller is its own subsystem so it can be
 * tuned live on its own; the commands here require both.
 */
public class FuelCommands {
  private final FeederSubsystem feeder;
  private final IntakeLauncherSubsystem intakeLauncher;

  // put default values for various fuel operations onto the dashboard
  // all methods here pull their values from the dashboard to allow
  // you to tune the values easily, and then replace the values in Constants.java
  // with your new values. For more information, see the Software Guide.
  private final TunableDouble intakingFeederVoltage = Tunables.addDouble("Intaking feeder roller value", INTAKING_FEEDER_VOLTAGE);
  private final TunableDouble intakingIntakeVoltage = Tunables.addDouble("Intaking intake roller value", INTAKING_INTAKE_VOLTAGE);
  private final TunableDouble launchingFeederVoltage = Tunables.addDouble("Launching feeder roller value", LAUNCHING_FEEDER_VOLTAGE);
  private final TunableDouble launchingLauncherVoltage = Tunables.addDouble("Launching launcher roller value", LAUNCHING_LAUNCHER_VOLTAGE);
  private final TunableDouble spinUpFeederVoltage = Tunables.addDouble("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);

  public FuelCommands(FeederSubsystem feeder, IntakeLauncherSubsystem intakeLauncher) {
    this.feeder = feeder;
    this.intakeLauncher = intakeLauncher;
  }

  // A method to set the rollers to values for intaking
  public void intake() {
    feeder.setVoltage(Volts.of(intakingFeederVoltage.get()));
    intakeLauncher.setVoltage(Volts.of(intakingIntakeVoltage.get()));
  }

  // A method to set the rollers to values for ejecting fuel out the intake. Uses
  // the same values as intaking, but in the opposite direction.
  public void eject() {
    feeder.setVoltage(Volts.of(-1 * intakingFeederVoltage.get()));
    intakeLauncher.setVoltage(Volts.of(-1 * intakingIntakeVoltage.get()));
  }

  // A method to set the rollers to values for launching.
  public void launch() {
    feeder.setVoltage(Volts.of(launchingFeederVoltage.get()));
    intakeLauncher.setVoltage(Volts.of(launchingLauncherVoltage.get()));
  }

  // A method to stop the rollers
  public void stop() {
    feeder.stop();
    intakeLauncher.stop();
  }

  // A method to spin up the launcher roller while spinning the feeder roller to
  // push Fuel away from the launcher
  public void spinUp() {
    feeder.setVoltage(Volts.of(spinUpFeederVoltage.get()));
    intakeLauncher.setVoltage(Volts.of(launchingLauncherVoltage.get()));
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
