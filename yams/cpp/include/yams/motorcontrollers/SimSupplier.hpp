// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

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
  virtual void SetInputVoltage(units::volt_t volts) = 0;

  /**
   * Get the simulated mechanism position.
   *
   * @return Mechanism position in degrees.
   */
  virtual units::degree_t GetMechanismPosition() = 0;

  /**
   * Get the simulated mechanism velocity.
   *
   * @return Mechanism velocity in degrees per second.
   */
  virtual units::degrees_per_second_t GetMechanismVelocity() = 0;

  /**
   * Get the simulated rotor position.
   *
   * @return Rotor position in degrees.
   */
  virtual units::degree_t GetRotorPosition() = 0;

  /**
   * Get the simulated rotor velocity.
   *
   * @return Rotor velocity in degrees per second.
   */
  virtual units::degrees_per_second_t GetRotorVelocity() = 0;

  /**
   * Get the simulated mechanism angular acceleration.
   *
   * @return Mechanism acceleration in degrees per second squared.
   */
  virtual units::degrees_per_second_squared_t GetMechanismAcceleration() = 0;

  /**
   * Get the simulated rotor angular acceleration.
   *
   * @return Rotor acceleration in degrees per second squared.
   */
  virtual units::degrees_per_second_squared_t GetRotorAcceleration() = 0;

  /**
   * Set the simulated mechanism position.
   *
   * @param angle Mechanism position to inject.
   */
  virtual void SetMechanismPosition(units::degree_t angle) = 0;

  /**
   * Set the simulated mechanism velocity.
   *
   * @param velocity Mechanism velocity to inject.
   */
  virtual void SetMechanismVelocity(units::degrees_per_second_t velocity) = 0;

  /**
   * Set the simulated rotor position.
   *
   * @param angle Rotor position to inject.
   */
  virtual void SetRotorPosition(units::degree_t angle) = 0;

  /**
   * Set the simulated rotor velocity.
   *
   * @param velocity Rotor velocity to inject.
   */
  virtual void SetRotorVelocity(units::degrees_per_second_t velocity) = 0;

  /**
   * Return true if the simulation watchdog has expired (sim state is stale).
   *
   * @return true if the watchdog has not been fed within the required period.
   */
  virtual bool IsWatchdogExpired() = 0;

  /** Feed the simulation watchdog to indicate the sim state is current. */
  virtual void FeedWatchdog() = 0;
};

}  // namespace yams::motorcontrollers
