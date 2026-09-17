// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.units.measure.Voltage;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Shared duty-cycle/voltage {@link Command} factories for command-layer mechanisms.
 *
 * <p>Implemented by every concrete commands2 mechanism (each of which extends the corresponding
 * {@code yams.core.mechanisms} class and supplies its own {@link Subsystem}) so that generic
 * open-loop control commands don't need to be duplicated per mechanism.
 */
public interface CommandMechanism {
  /**
   * Motor controller driving the mechanism.
   *
   * @return {@link SmartMotorController} for the mechanism.
   */
  SmartMotorController getMotorController();

  /**
   * Subsystem the mechanism's commands should require.
   *
   * @return {@link Subsystem} for the mechanism.
   */
  Subsystem getSubsystem();

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set.
   * @return {@link Command}
   */
  default Command set(double dutycycle) {
    SmartMotorController smc = getMotorController();
    Subsystem subsystem = getSubsystem();
    return Commands
        .startRun(smc::stopClosedLoopController, () -> smc.setDutyCycle(dutycycle), subsystem)
        .finallyDo(smc::startClosedLoopController)
        .withName(subsystem.getName() + " SetDutyCycle");
  }

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set via a {@link Supplier}.
   * @return {@link Command}
   */
  default Command set(Supplier<Double> dutycycle) {
    SmartMotorController smc = getMotorController();
    Subsystem subsystem = getSubsystem();
    return Commands
        .startRun(
            smc::stopClosedLoopController, () -> smc.setDutyCycle(dutycycle.get()), subsystem)
        .finallyDo(smc::startClosedLoopController)
        .withName(subsystem.getName() + " SetDutyCycle Supplier");
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param volts {@link Voltage} of the {@link SmartMotorController} to set.
   * @return {@link Command}
   */
  default Command setVoltage(Voltage volts) {
    SmartMotorController smc = getMotorController();
    Subsystem subsystem = getSubsystem();
    return Commands.startRun(smc::stopClosedLoopController, () -> smc.setVoltage(volts), subsystem)
        .finallyDo(smc::startClosedLoopController)
        .withName(subsystem.getName() + " SetVoltage");
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param volts {@link Voltage} of the {@link SmartMotorController} to set, via a
   *              {@link Supplier}.
   * @return {@link Command}
   */
  default Command setVoltage(Supplier<Voltage> volts) {
    SmartMotorController smc = getMotorController();
    Subsystem subsystem = getSubsystem();
    return Commands
        .startRun(smc::stopClosedLoopController, () -> smc.setVoltage(volts.get()), subsystem)
        .finallyDo(smc::startClosedLoopController)
        .withName(subsystem.getName() + " SetVoltage Supplier");
  }
}
