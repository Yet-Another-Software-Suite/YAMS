// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.mechanisms;

import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.Voltage;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Shared duty-cycle/voltage {@link Command} factories for command-layer mechanisms.
 */
public interface CommandMechanism {
  /**
   * Motor controller driving the mechanism.
   *
   * @return {@link SmartMotorController} for the mechanism.
   */
  SmartMotorController getMotorController();

  /**
   * Mechanism the mechanism's commands should require.
   *
   * @return {@link Mechanism} for the mechanism.
   */
  Mechanism getMechanism();

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set.
   * @return {@link Command}
   */
  default Command set(double dutycycle) {
    SmartMotorController smc = getMotorController();
    Mechanism mechanism = getMechanism();
    return mechanism.run(coroutine -> {
      smc.stopClosedLoopController();
      while (true) {
        smc.setDutyCycle(dutycycle);
        coroutine.yield();
      }
    }).whenCanceled(smc::startClosedLoopController).named(mechanism.getName() + " SetDutyCycle");
  }

  /**
   * Set the DutyCycle of the {@link SmartMotorController}.
   *
   * @param dutycycle [-1,1] to set via a {@link Supplier}.
   * @return {@link Command}
   */
  default Command set(Supplier<Double> dutycycle) {
    SmartMotorController smc = getMotorController();
    Mechanism mechanism = getMechanism();
    return mechanism.run(coroutine -> {
      smc.stopClosedLoopController();
      while (true) {
        smc.setDutyCycle(dutycycle.get());
        coroutine.yield();
      }
    }).whenCanceled(smc::startClosedLoopController).named(mechanism.getName() + " SetDutyCycle Supplier");
  }

  /**
   * Set the voltage of the {@link SmartMotorController}.
   *
   * @param volts {@link Voltage} of the {@link SmartMotorController} to set.
   * @return {@link Command}
   */
  default Command setVoltage(Voltage volts) {
    SmartMotorController smc = getMotorController();
    Mechanism mechanism = getMechanism();
    return mechanism.run(coroutine -> {
      smc.stopClosedLoopController();
      while (true) {
        smc.setVoltage(volts);
        coroutine.yield();
      }
    }).whenCanceled(smc::startClosedLoopController).named(mechanism.getName() + " SetVoltage");
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
    Mechanism mechanism = getMechanism();
    return mechanism.run(coroutine -> {
      smc.stopClosedLoopController();
      while (true) {
        smc.setVoltage(volts.get());
        coroutine.yield();
      }
    }).whenCanceled(smc::startClosedLoopController).named(mechanism.getName() + " SetVoltage Supplier");
  }
}
