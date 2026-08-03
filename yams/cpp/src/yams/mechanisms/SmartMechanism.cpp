// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/SmartMechanism.hpp"

#include <wpi/commands2/Commands.hpp>

namespace yams::mechanisms {

// ---- Open-loop command factories --------------------------------------------

wpi::cmd::CommandPtr SmartMechanism::Set(double dutycycle) {
  return wpi::cmd::StartRun([this] { m_smc->StopClosedLoopController(); },
                             [this, dutycycle] { m_smc->SetDutyCycle(dutycycle); }, {m_subsystem})
      .FinallyDo([this](bool) { m_smc->StartClosedLoopController(); })
      .WithName(m_subsystem->GetName() + " SetDutyCycle");
}

wpi::cmd::CommandPtr SmartMechanism::Set(std::function<double()> dutycycle) {
  return wpi::cmd::StartRun([this] { m_smc->StopClosedLoopController(); },
                             [this, dutycycle] { m_smc->SetDutyCycle(dutycycle()); }, {m_subsystem})
      .FinallyDo([this](bool) { m_smc->StartClosedLoopController(); })
      .WithName(m_subsystem->GetName() + " SetDutyCycle Supplier");
}

wpi::cmd::CommandPtr SmartMechanism::SetVoltage(wpi::units::volt_t volts) {
  return wpi::cmd::StartRun([this] { m_smc->StopClosedLoopController(); },
                             [this, volts] { m_smc->SetVoltage(volts); }, {m_subsystem})
      .FinallyDo([this](bool) { m_smc->StartClosedLoopController(); })
      .WithName(m_subsystem->GetName() + " SetVoltage");
}

wpi::cmd::CommandPtr SmartMechanism::SetVoltage(std::function<wpi::units::volt_t()> volts) {
  return wpi::cmd::StartRun([this] { m_smc->StopClosedLoopController(); },
                             [this, volts] { m_smc->SetVoltage(volts()); }, {m_subsystem})
      .FinallyDo([this](bool) { m_smc->StartClosedLoopController(); })
      .WithName(m_subsystem->GetName() + " SetVoltage Supplier");
}

// ---- Direct setpoint helpers ------------------------------------------------

void SmartMechanism::SetMechanismVelocitySetpoint(wpi::units::turns_per_second_t velocity) {
  m_smc->StartClosedLoopController();
  m_smc->SetVelocity(velocity);
}

void SmartMechanism::SetMeasurementVelocitySetpoint(wpi::units::meters_per_second_t velocity) {
  m_smc->StartClosedLoopController();
  m_smc->SetVelocity(velocity);
}

void SmartMechanism::SetMechanismPositionSetpoint(wpi::units::turn_t angle) {
  m_smc->StartClosedLoopController();
  m_smc->SetPosition(angle);
}

void SmartMechanism::SetMeasurementPositionSetpoint(wpi::units::meter_t distance) {
  m_smc->StartClosedLoopController();
  m_smc->SetPosition(distance);
}

void SmartMechanism::SetVoltageSetpoint(wpi::units::volt_t voltage) {
  m_smc->StopClosedLoopController();
  m_smc->SetVoltage(voltage);
}

void SmartMechanism::SetDutyCycleSetpoint(double dutycycle) {
  m_smc->StopClosedLoopController();
  m_smc->SetDutyCycle(dutycycle);
}

// ---- Accessors --------------------------------------------------------------

motorcontrollers::SmartMotorController* SmartMechanism::GetMotorController() { return m_smc; }

std::optional<wpi::units::turn_t> SmartMechanism::GetMechanismSetpoint() {
  return m_smc->GetMechanismPositionSetpoint();
}

wpi::Mechanism2d* SmartMechanism::GetMechanismWindow() {
  if (m_mechanismWindow) {
    return &(*m_mechanismWindow);
  }
  return nullptr;
}

}  // namespace yams::mechanisms
