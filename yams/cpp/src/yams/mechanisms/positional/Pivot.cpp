// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Pivot.h"

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>

#include <cmath>
#include <string>

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Pivot::Pivot(const config::PivotConfig& config) : SmartMechanism(), SmartPositionalMechanism(), m_pivotConfig{config} {
  m_smc = config.GetMotorController();
  m_subsystem = config.GetSubsystem();

  if (!config.GetTelemetryName().empty()) {
    m_name = config.GetTelemetryName();
  }

  // Apply angular soft limits to the motor controller when configured.
  if (auto minA = config.GetMinAngle()) {
    m_smc->SetMechanismLowerLimit(*minA);
  }
  if (auto maxA = config.GetMaxAngle()) {
    m_smc->SetMechanismUpperLimit(*maxA);
  }

  // Seed the encoder from the configured starting angle.
  if (auto startA = config.GetStartingAngle()) {
    m_smc->SetEncoderPosition(*startA);
  }

  // Build a Mechanism2d window for visualisation.
  // Use a fixed ligament length of 0.5 m to represent the rotating assembly.
  constexpr double kLigamentLength = 0.5;
  m_mechanismWindow.emplace(1.2, 1.2);
  m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + " Root", 0.6, 0.6);
  m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
      m_name + " Arm", kLigamentLength, GetAngle(), 6, frc::Color8Bit{frc::Color::kGreen});

  frc::SmartDashboard::PutData(m_name, &(*m_mechanismWindow));
}

// ---- SmartMechanism overrides -----------------------------------------------

void Pivot::SimIterate() { m_smc->SimIterate(); }

void Pivot::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void Pivot::VisualizationUpdate() {
  if (m_mechanismLigament) {
    m_mechanismLigament->SetAngle(GetAngle());
  }
}

std::string Pivot::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

frc2::Trigger Pivot::Max() {
  return frc2::Trigger{[this] {
    return GetAngle() >= m_pivotConfig.GetMaxAngle().value_or(units::degree_t{36000});
  }};
}

frc2::Trigger Pivot::Min() {
  return frc2::Trigger{[this] {
    return GetAngle() <= m_pivotConfig.GetMinAngle().value_or(units::degree_t{-36000});
  }};
}

frc2::CommandPtr Pivot::SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                              units::second_t duration) {
  auto routine = m_smc->SysId(maxVoltage, step, duration);
  return frc2::cmd::Sequence(routine.Quasistatic(frc2::sysid::Direction::kForward),
                             routine.Quasistatic(frc2::sysid::Direction::kReverse),
                             routine.Dynamic(frc2::sysid::Direction::kForward),
                             routine.Dynamic(frc2::sysid::Direction::kReverse));
}

// ---- Pivot-specific interface -----------------------------------------------

frc2::CommandPtr Pivot::GoToAngle(units::degree_t angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " GoToAngle");
}

frc2::CommandPtr Pivot::GoToAngle(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " GoToAngle Supplier");
}

units::degree_t Pivot::GetAngle() const { return m_smc->GetMechanismPosition(); }

bool Pivot::IsAtAngle(units::degree_t target, units::degree_t tolerance) const {
  return std::abs(GetAngle().value() - target.value()) <= tolerance.value();
}

frc2::Trigger Pivot::AtAngle(units::degree_t target, units::degree_t tolerance) {
  return frc2::Trigger{[this, target, tolerance] { return IsAtAngle(target, tolerance); }};
}

}  // namespace yams::mechanisms::positional
