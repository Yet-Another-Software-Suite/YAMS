// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Pivot.hpp"

#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
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

Pivot::Pivot(const config::PivotConfig& config)
    : SmartPositionalMechanism(), m_pivotConfig{config} {
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

void Pivot::SimIterate() {
  if (auto* ss = m_smc->GetSimSupplier()) {
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->FeedWatchdog();

    if (m_pivotConfig.GetMinAngle() && ss->GetMechanismVelocity().value() < 0.0 &&
        GetAngle() < *m_pivotConfig.GetMinAngle()) {
      m_smc->SetEncoderPosition(*m_pivotConfig.GetMinAngle());
    }
    if (m_pivotConfig.GetMaxAngle() && ss->GetMechanismVelocity().value() > 0.0 &&
        GetAngle() > *m_pivotConfig.GetMaxAngle()) {
      m_smc->SetEncoderPosition(*m_pivotConfig.GetMaxAngle());
    }
    frc::sim::RoboRioSim::SetVInVoltage(
        frc::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    VisualizationUpdate();
  }
}

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

frc2::CommandPtr Pivot::Run(units::degree_t angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " Run");
}

frc2::CommandPtr Pivot::Run(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

frc2::CommandPtr Pivot::RunTo(units::degree_t angle, units::degree_t tolerance) {
  frc2::Trigger near = IsNear(angle, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

frc2::CommandPtr Pivot::RunTo(std::function<units::degree_t()> angle, units::degree_t tolerance) {
  units::degree_t target = angle();
  frc2::Trigger near = IsNear(target, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, target] { SetMechanismPositionSetpoint(target); }, {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

frc2::Trigger Pivot::Gte(units::degree_t angle) {
  return frc2::Trigger{[this, angle] { return GetAngle() >= angle; }};
}

frc2::Trigger Pivot::Lte(units::degree_t angle) {
  return frc2::Trigger{[this, angle] { return GetAngle() <= angle; }};
}

frc2::Trigger Pivot::Between(units::degree_t start, units::degree_t end) {
  return Gte(start) && (Lte(end));
}

frc2::Trigger Pivot::IsNear(units::degree_t angle, units::degree_t within) {
  return frc2::Trigger{[this, angle, within] {
    return std::abs(GetAngle().value() - angle.value()) <= within.value();
  }};
}

const config::PivotConfig& Pivot::GetConfig() const { return m_pivotConfig; }

frc::Translation3d Pivot::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return frc::Translation3d{units::meter_t{m_mechanismLigament->GetLength()},
                              frc::Rotation3d{0_rad, 0_rad, units::radian_t{GetAngle()}}};
  }
  return frc::Translation3d{};
}

void Pivot::SetAngle(units::degree_t angle) { SetMechanismPositionSetpoint(angle); }

units::degree_t Pivot::GetAngle() const { return m_smc->GetMechanismPosition(); }

}  // namespace yams::mechanisms::positional
