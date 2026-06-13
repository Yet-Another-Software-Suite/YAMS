// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>
#include <units/voltage.h>

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
  frc2::CommandPtr Set(double dutycycle);

  /**
   * Run the motor at a supplier-provided duty-cycle.
   *
   * @param dutycycle Supplier returning a [-1, 1] value each loop.
   * @return CommandPtr that requires m_subsystem.
   */
  frc2::CommandPtr Set(std::function<double()> dutycycle);

  /**
   * Run the motor at a fixed voltage.
   *
   * @param volts Voltage to apply.
   * @return CommandPtr that requires m_subsystem.
   */
  frc2::CommandPtr SetVoltage(units::volt_t volts);

  /**
   * Run the motor at a supplier-provided voltage.
   *
   * @param volts Supplier returning a voltage each loop.
   * @return CommandPtr that requires m_subsystem.
   */
  frc2::CommandPtr SetVoltage(std::function<units::volt_t()> volts);

  // ---- Direct setpoint helpers (no command wrapper) -------------------------

  /** Start closed-loop control and set a mechanism angular velocity setpoint. */
  void SetMechanismVelocitySetpoint(units::turns_per_second_t velocity);

  /** Start closed-loop control and set a linear measurement velocity setpoint. */
  void SetMeasurementVelocitySetpoint(units::meters_per_second_t velocity);

  /** Start closed-loop control and set a mechanism angle setpoint. */
  void SetMechanismPositionSetpoint(units::turn_t angle);

  /** Start closed-loop control and set a linear measurement position setpoint. */
  void SetMeasurementPositionSetpoint(units::meter_t distance);

  /**
   * Stop the closed-loop controller and apply a voltage immediately.
   *
   * @param voltage Voltage to apply.
   */
  void SetVoltageSetpoint(units::volt_t voltage);

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
  std::optional<units::turn_t> GetMechanismSetpoint();

  /**
   * Get a pointer to the Mechanism2d window, or nullptr if not initialised.
   *
   * @return Pointer to the Mechanism2d window.
   */
  frc::Mechanism2d* GetMechanismWindow();

 protected:
  /** Subsystem that this mechanism belongs to (provides command requirements). */
  frc2::SubsystemBase* m_subsystem{nullptr};

  /** Motor controller driving this mechanism. */
  motorcontrollers::SmartMotorController* m_smc{nullptr};

  /** Optional 2D visualisation window published to SmartDashboard. */
  std::optional<frc::Mechanism2d> m_mechanismWindow;
};

}  // namespace yams::mechanisms
