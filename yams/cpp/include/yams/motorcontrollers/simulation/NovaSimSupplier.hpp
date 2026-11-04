// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <thrifty/core/MotorControlRequest.h>
#include <thrifty/nova/Nova.h>

#include <memory>
#include <optional>
#include <wpi/math/trajectory/TrapezoidProfile.hpp>
#include <wpi/units/time.hpp>

#include "yams/motorcontrollers/SimSupplier.hpp"

namespace yams::motorcontrollers::simulation {

/**
 * A simulated Thrifty Nova driving a mechanism's SimSupplier. ThriftyLib's simulation ignores the
 * Nova's closed loop, so this runs it as the Nova would, with the gains read back from the Nova:
 * the voltage position and velocity loops with the PID slot's gains and its F times the target, and
 * the motion profile with its slot's kS, kV, kA and kG. It keeps within the Nova's soft limits,
 * and, with its absolute wrapping on, takes a position the short way, one mechanism rotation apart.
 * Targets and gains are in mechanism rotations, as on a Nova with its sensor-to-mechanism ratio
 * set, and the integral and derivative gains are per second, as the Nova's are. Open loop output,
 * a duty cycle or a voltage, is passed on to the mechanism.
 */
class NovaSimSupplier : public SimSupplier {
 public:
  /**
   * Create a simulated Nova.
   *
   * @param mechanism Mechanism the Nova drives.
   * @param nova      Nova whose gains are simulated.
   * @param period    Simulation period.
   */
  NovaSimSupplier(std::shared_ptr<SimSupplier> mechanism, thrifty::Nova& nova,
                  wpi::units::second_t period);

  /**
   * Run a closed loop control request, as the Nova does when it is sent one. The integral, and the
   * motion profile, carry on while requests of the same type are sent.
   *
   * @param request Voltage position, voltage velocity or motion profile request.
   */
  void SetControlRequest(const thrifty::MotorControlRequest& request);

  /** @return The closed loop request being run, or empty in open loop. */
  const std::optional<thrifty::MotorControlRequest>& GetControlRequest() const;

  /** @return Duty cycle applied to the motor, from -1 to 1. */
  double GetDutyCycle() const;

  void UpdateSim() override;
  bool GetUpdatedSim() override;
  void FeedUpdateSim() override;
  void StarveUpdateSim() override;
  bool IsInputFed() override;
  void FeedInput() override;
  void StarveInput() override;
  /** Open loop: the Nova stops its closed loop. */
  void SetMechanismStatorDutyCycle(double dutyCycle) override;
  wpi::units::volt_t GetMechanismSupplyVoltage() override;
  wpi::units::volt_t GetMechanismStatorVoltage() override;
  /** Open loop: the Nova stops its closed loop. */
  void SetMechanismStatorVoltage(wpi::units::volt_t volts) override;
  wpi::units::turn_t GetMechanismPosition() override;
  void SetMechanismPosition(wpi::units::turn_t angle) override;
  wpi::units::turn_t GetRotorPosition() override;
  wpi::units::turns_per_second_t GetMechanismVelocity() override;
  void SetMechanismVelocity(wpi::units::turns_per_second_t velocity) override;
  wpi::units::turns_per_second_t GetRotorVelocity() override;
  wpi::units::turns_per_second_squared_t GetMechanismAcceleration() override;
  wpi::units::turns_per_second_squared_t GetRotorAcceleration() override;
  void SetRotorPosition(wpi::units::turn_t angle) override;
  void SetRotorVelocity(wpi::units::turns_per_second_t velocity) override;
  wpi::units::ampere_t GetStatorCurrent() override;
  wpi::units::ampere_t GetSupplyCurrent() override;

 private:
  /** Apply a voltage to the motor, within the supply voltage. */
  void ApplyVoltage(double volts);

  std::shared_ptr<SimSupplier> m_mechanism;
  thrifty::Nova& m_nova;
  double m_periodSeconds;
  std::optional<thrifty::MotorControlRequest> m_request;
  double m_dutyCycle{0.0};
  /** Sum of the closed loop error over every Nova loop. */
  double m_integral{0.0};
  /** Closed loop error of the last period, or empty after a new type of request. */
  std::optional<double> m_lastError;
  /** Point the motion profile reached, or empty before it starts. */
  std::optional<wpi::math::TrapezoidProfile<wpi::units::turns>::State> m_profileState;
};

}  // namespace yams::motorcontrollers::simulation
