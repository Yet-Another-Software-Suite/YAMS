// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/smartdashboard/Mechanism2d.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include <functional>
#include <optional>
#include <string>

#include "yams/motorcontrollers/SmartMotorController.hpp"

namespace yams::mechanisms {

/**
 * Abstract base class for all YAMS smart mechanisms.
 *
 * Provides common open-loop command factories and setpoint helpers that
 * delegate to the underlying SmartMotorController.
 */
class SmartMechanism {
 public:
  SmartMechanism() {};
  virtual ~SmartMechanism() = default;

  // ---- Pure virtual interface -----------------------------------------------

  /** Advance the mechanism's simulation by one loop iteration. */
  virtual void SimIterate() = 0;

  /** Publish telemetry to NetworkTables / SmartDashboard. */
  virtual void UpdateTelemetry() = 0;

  /** Update the Mechanism2d visualization state. */
  virtual void VisualizationUpdate() = 0;

  /** Get the human-readable name of this mechanism. */
  virtual std::string GetName() const = 0;

  // ---- Open-loop command factories ------------------------------------------

  /**
   * Run the motor at a fixed duty-cycle.  Stops the closed-loop controller
   * while the command is running and restarts it on finish.
   *
   * @param dutycycle [-1, 1] duty cycle to apply.
   * @return CommandPtr that requires m_subsystem.
   */
  wpi::cmd::CommandPtr Set(double dutycycle);

  /**
   * Run the motor at a supplier-provided duty-cycle.
   *
   * @param dutycycle Supplier returning a [-1, 1] value each loop.
   * @return CommandPtr that requires m_subsystem.
   */
  wpi::cmd::CommandPtr Set(std::function<double()> dutycycle);

  /**
   * Run the motor at a fixed voltage.
   *
   * @param volts Voltage to apply.
   * @return CommandPtr that requires m_subsystem.
   */
  wpi::cmd::CommandPtr SetVoltage(wpi::units::volt_t volts);

  /**
   * Run the motor at a supplier-provided voltage.
   *
   * @param volts Supplier returning a voltage each loop.
   * @return CommandPtr that requires m_subsystem.
   */
  wpi::cmd::CommandPtr SetVoltage(std::function<wpi::units::volt_t()> volts);

  // ---- Direct setpoint helpers (no command wrapper) -------------------------

  /** Start closed-loop control and set a mechanism angular velocity setpoint. */
  void SetMechanismVelocitySetpoint(wpi::units::turns_per_second_t velocity);

  /** Start closed-loop control and set a linear measurement velocity setpoint. */
  void SetMeasurementVelocitySetpoint(wpi::units::meters_per_second_t velocity);

  /** Start closed-loop control and set a mechanism angle setpoint. */
  void SetMechanismPositionSetpoint(wpi::units::turn_t angle);

  /** Start closed-loop control and set a linear measurement position setpoint. */
  void SetMeasurementPositionSetpoint(wpi::units::meter_t distance);

  /**
   * Stop the closed-loop controller and apply a voltage immediately.
   *
   * @param voltage Voltage to apply.
   */
  void SetVoltageSetpoint(wpi::units::volt_t voltage);

  /**
   * Stop the closed-loop controller and apply a duty cycle immediately.
   *
   * @param dutycycle [-1, 1] duty cycle.
   */
  void SetDutyCycleSetpoint(double dutycycle);

  // ---- Accessors ------------------------------------------------------------

  /** Get the underlying SmartMotorController. */
  motorcontrollers::SmartMotorController* GetMotorController();

  /**
   * Get the current mechanism angle setpoint if one has been set.
   *
   * @return Optional angle setpoint.
   */
  std::optional<wpi::units::turn_t> GetMechanismSetpoint();

  /**
   * Get a pointer to the Mechanism2d window, or nullptr if not initialised.
   *
   * @return Pointer to the Mechanism2d window.
   */
  wpi::Mechanism2d* GetMechanismWindow();

 protected:
  /** Subsystem that this mechanism belongs to (provides command requirements). */
  wpi::cmd::SubsystemBase* m_subsystem{nullptr};

  /** Motor controller driving this mechanism. */
  motorcontrollers::SmartMotorController* m_smc{nullptr};

  /** Optional 2D visualisation window published to SmartDashboard. */
  std::optional<wpi::Mechanism2d> m_mechanismWindow;
};

}  // namespace yams::mechanisms
