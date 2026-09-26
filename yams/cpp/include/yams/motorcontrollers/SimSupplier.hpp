// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/units/angle.hpp>
#include <wpi/units/angular_acceleration.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

namespace yams::motorcontrollers {

/**
 * Abstract interface for simulation state providers.
 *
 * Implementors feed simulated mechanism and rotor positions/velocities back into
 * a SmartMotorController during simulation iterations.
 */
class SimSupplier {
 public:
  virtual ~SimSupplier() = default;

  /** Advance the simulation by one loop iteration. */
  virtual void UpdateSim() = 0;

  /**
   * Directly set the motor input voltage, bypassing the duty-cycle supplier.
   * Used when the hardware sim state (e.g. TalonFX) provides voltage directly.
   *
   * @param volts Input voltage to inject.
   */
  virtual void SetInputVoltage(wpi::units::volt_t volts) = 0;

  /**
   * Get the simulated mechanism position.
   *
   * @return Mechanism position in turns.
   */
  virtual wpi::units::turn_t GetMechanismPosition() = 0;

  /**
   * Get the simulated mechanism velocity.
   *
   * @return Mechanism velocity in turns per second.
   */
  virtual wpi::units::turns_per_second_t GetMechanismVelocity() = 0;

  /**
   * Get the simulated rotor position.
   *
   * @return Rotor position in turns.
   */
  virtual wpi::units::turn_t GetRotorPosition() = 0;

  /**
   * Get the simulated rotor velocity.
   *
   * @return Rotor velocity in turns per second.
   */
  virtual wpi::units::turns_per_second_t GetRotorVelocity() = 0;

  /**
   * Get the simulated mechanism angular acceleration.
   *
   * @return Mechanism acceleration in turns per second squared.
   */
  virtual wpi::units::turns_per_second_squared_t GetMechanismAcceleration() = 0;

  /**
   * Get the simulated rotor angular acceleration.
   *
   * @return Rotor acceleration in turns per second squared.
   */
  virtual wpi::units::turns_per_second_squared_t GetRotorAcceleration() = 0;

  /**
   * Set the simulated mechanism position.
   *
   * @param angle Mechanism position to inject.
   */
  virtual void SetMechanismPosition(wpi::units::turn_t angle) = 0;

  /**
   * Set the simulated mechanism velocity.
   *
   * @param velocity Mechanism velocity to inject.
   */
  virtual void SetMechanismVelocity(wpi::units::turns_per_second_t velocity) = 0;

  /**
   * Set the simulated rotor position.
   *
   * @param angle Rotor position to inject.
   */
  virtual void SetRotorPosition(wpi::units::turn_t angle) = 0;

  /**
   * Set the simulated rotor velocity.
   *
   * @param velocity Rotor velocity to inject.
   */
  virtual void SetRotorVelocity(wpi::units::turns_per_second_t velocity) = 0;

  /**
   * Return true if the simulation watchdog has expired (sim state is stale).
   *
   * @return true if the watchdog has not been fed within the required period.
   */
  virtual bool IsWatchdogExpired() = 0;

  /** Feed the simulation watchdog to indicate the sim state is current. */
  virtual void FeedWatchdog() = 0;

  /** Stale the simulation watchdog so the next SimIterate triggers an update. */
  virtual void StarveWatchdog() = 0;

  /**
   * Get the current draw from the simulated motor.
   *
   * @return Current draw in amperes.
   */
  virtual wpi::units::ampere_t GetCurrentDrawAmps() = 0;

  /**
   * Get the supply voltage available to the simulated motor controller (bus voltage).
   *
   * @return Supply voltage in volts.
   */
  virtual wpi::units::volt_t GetMechanismSupplyVoltage() = 0;

  /**
   * Get the last voltage applied to the simulated motor.
   *
   * @return Stator voltage in volts.
   */
  virtual wpi::units::volt_t GetMechanismStatorVoltage() = 0;

  /**
   * Set the motor input voltage directly (e.g. from a hardware sim state).
   *
   * @param volts Voltage to apply.
   */
  virtual void SetMechanismStatorVoltage(wpi::units::volt_t volts) = 0;
};

}  // namespace yams::motorcontrollers
