// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Elevator.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/Commands.h>
#include <units/length.h>

#include <cmath>
#include <string>

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Elevator::Elevator(const config::ElevatorConfig& config) : SmartPositionalMechanism(), m_elevatorConfig{config} {
  m_smc = config.GetMotorController();
  m_subsystem = config.GetSubsystem();

  if (!config.GetTelemetryName().empty()) {
    m_name = config.GetTelemetryName();
  }

  // Apply soft limits to the motor controller when configured.
  if (auto minH = config.GetMinHeight()) {
    m_smc->SetMeasurementLowerLimit(*minH);
  }
  if (auto maxH = config.GetMaxHeight()) {
    m_smc->SetMeasurementUpperLimit(*maxH);
  }

  // Seed the encoder from the configured starting height.
  if (auto startH = config.GetStartingHeight()) {
    m_smc->SetEncoderPosition(*startH);
  }

  // Build a Mechanism2d window for visualisation.
  double windowHeight = 3.0;  // metres — generous default
  if (auto maxH = config.GetMaxHeight()) {
    windowHeight = maxH->value() + 0.2;
  }
  m_mechanismWindow.emplace(1.0, windowHeight);
  m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + " Root", 0.5, 0.0);
  m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
      m_name + " Carriage", GetHeight().value(), 90_deg, 6, frc::Color8Bit{frc::Color::kOrange});

  frc::SmartDashboard::PutData(m_name, &(*m_mechanismWindow));
}

// ---- SmartMechanism overrides -----------------------------------------------

void Elevator::SimIterate() { m_smc->SimIterate(); }

void Elevator::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void Elevator::VisualizationUpdate() {
  if (m_mechanismLigament) {
    m_mechanismLigament->SetLength(GetHeight().value());
  }
}

std::string Elevator::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

frc2::Trigger Elevator::Max() {
  return frc2::Trigger{[this] {
    return GetHeight() >= m_elevatorConfig.GetMaxHeight().value_or(units::meter_t{99});
  }};
}

frc2::Trigger Elevator::Min() {
  return frc2::Trigger{[this] {
    return GetHeight() <= m_elevatorConfig.GetMinHeight().value_or(units::meter_t{-99});
  }};
}

frc2::CommandPtr Elevator::SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                                 units::second_t duration) {
  auto routine = m_smc->SysId(maxVoltage, step, duration);
  return frc2::cmd::Sequence(routine.Quasistatic(frc2::sysid::Direction::kForward),
                             routine.Quasistatic(frc2::sysid::Direction::kReverse),
                             routine.Dynamic(frc2::sysid::Direction::kForward),
                             routine.Dynamic(frc2::sysid::Direction::kReverse));
}

// ---- Elevator-specific interface --------------------------------------------

frc2::CommandPtr Elevator::GoToHeight(units::meter_t height) {
  return frc2::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height); }, {m_subsystem})
      .WithName(m_name + " GoToHeight");
}

frc2::CommandPtr Elevator::GoToHeight(std::function<units::meter_t()> height) {
  return frc2::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height()); }, {m_subsystem})
      .WithName(m_name + " GoToHeight Supplier");
}

units::meter_t Elevator::GetHeight() const { return m_smc->GetMeasurementPosition(); }

bool Elevator::IsAtHeight(units::meter_t target, units::meter_t tolerance) const {
  return std::abs(GetHeight().value() - target.value()) <= tolerance.value();
}

frc2::Trigger Elevator::AtHeight(units::meter_t target, units::meter_t tolerance) {
  return frc2::Trigger{[this, target, tolerance] { return IsAtHeight(target, tolerance); }};
}

}  // namespace yams::mechanisms::positional
