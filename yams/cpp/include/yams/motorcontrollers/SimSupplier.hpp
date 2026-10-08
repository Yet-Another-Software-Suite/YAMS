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

  /**
   * Advance the simulation by one loop iteration.
   *
   * When the input has not been fed this loop (see FeedInput()), the motor duty cycle times the
   * supply voltage is applied as the input and the battery voltage is updated.  The physics are
   * only stepped if the simulation has not already been updated this loop (see GetUpdatedSim());
   * stepping marks it updated, so a mechanism and its motor controller calling UpdateSim() in the
   * same loop step the physics once.
   */
  virtual void UpdateSim() = 0;

  /**
   * Whether the simulation has already been stepped this loop.
   *
   * @return true if UpdateSim() stepped the physics since the last StarveUpdateSim().
   */
  virtual bool GetUpdatedSim() = 0;

  /** Mark the simulation as stepped for this loop. */
  virtual void FeedUpdateSim() = 0;

  /** Clear the stepped flag so the next UpdateSim() steps the physics again. */
  virtual void StarveUpdateSim() = 0;

  /**
   * Whether an input voltage was supplied directly this loop (e.g. from a hardware sim state).
   *
   * @return true if the input was fed since the simulation last stepped.
   */
  virtual bool IsInputFed() = 0;

  /** Mark the input as supplied directly for this loop. */
  virtual void FeedInput() = 0;

  /** Clear the input-fed flag so UpdateSim() falls back to the duty-cycle supplier. */
  virtual void StarveInput() = 0;

  /**
   * Apply a duty cycle of the supply voltage as the motor input (feeds the input).
   *
   * @param dutyCycle Duty cycle in [-1, 1].
   */
  virtual void SetMechanismStatorDutyCycle(double dutyCycle) = 0;

  /**
   * Get the supply voltage available to the simulated motor controller (bus voltage).
   *
   * @return Supply voltage in volts.
   */
  virtual wpi::units::volt_t GetMechanismSupplyVoltage() = 0;

  /**
   * Get the voltage applied to the simulated motor.
   *
   * @return Stator voltage in volts.
   */
  virtual wpi::units::volt_t GetMechanismStatorVoltage() = 0;

  /**
   * Set the motor input voltage directly, e.g. from a hardware sim state (feeds the input).
   *
   * @param volts Voltage to apply.
   */
  virtual void SetMechanismStatorVoltage(wpi::units::volt_t volts) = 0;

  /**
   * Get the simulated mechanism position.
   *
   * @return Mechanism position in turns.
   */
  virtual wpi::units::turn_t GetMechanismPosition() = 0;

  /**
   * Set the simulated mechanism position.
   *
   * @param angle Mechanism position to inject.
   */
  virtual void SetMechanismPosition(wpi::units::turn_t angle) = 0;

  /**
   * Get the simulated rotor position.
   *
   * @return Rotor position in turns.
   */
  virtual wpi::units::turn_t GetRotorPosition() = 0;

  /**
   * Get the simulated mechanism velocity.
   *
   * @return Mechanism velocity in turns per second.
   */
  virtual wpi::units::turns_per_second_t GetMechanismVelocity() = 0;

  /**
   * Set the simulated mechanism velocity.
   *
   * @param velocity Mechanism velocity to inject.
   */
  virtual void SetMechanismVelocity(wpi::units::turns_per_second_t velocity) = 0;

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
   * Get the simulated stator (motor) current.
   *
   * @return Stator current in amperes.
   */
  virtual wpi::units::ampere_t GetStatorCurrent() = 0;

  /**
   * Get the simulated supply (battery) current: duty cycle times stator current, filtered by a
   * 0.1 s single-pole IIR filter.
   *
   * @return Supply current in amperes.
   */
  virtual wpi::units::ampere_t GetSupplyCurrent() = 0;
};

}  // namespace yams::motorcontrollers
