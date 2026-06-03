// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Arm.h"

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

Arm::Arm(const config::ArmConfig& config) : SmartPositionalMechanism(), m_armConfig{config} {
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
  double armLengthM = config.GetArmLength().value_or(units::meter_t{1.0}).value();
  m_mechanismWindow.emplace(armLengthM * 2.0 + 0.4, armLengthM * 2.0 + 0.4);
  m_mechanismRoot =
      m_mechanismWindow->GetRoot(m_name + " Root", armLengthM + 0.2, armLengthM + 0.2);
  m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
      m_name + " Joint", armLengthM, GetAngle(), 6, frc::Color8Bit{frc::Color::kAqua});

  frc::SmartDashboard::PutData(m_name, &(*m_mechanismWindow));
}

// ---- SmartMechanism overrides -----------------------------------------------

void Arm::SimIterate() {
  if (auto* ss = m_smc->GetSimSupplier()) {
    ss->UpdateSim();
    m_smc->SetEncoderPosition(ss->GetMechanismPosition());
    m_smc->SetEncoderVelocity(ss->GetMechanismVelocity());
    ss->FeedWatchdog();
  } else {
    m_smc->SimIterate();
  }
}

void Arm::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void Arm::VisualizationUpdate() {
  if (m_mechanismLigament) {
    m_mechanismLigament->SetAngle(GetAngle());
  }
}

std::string Arm::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

frc2::Trigger Arm::Max() {
  return frc2::Trigger{
      [this] { return GetAngle() >= m_armConfig.GetMaxAngle().value_or(units::degree_t{36000}); }};
}

frc2::Trigger Arm::Min() {
  return frc2::Trigger{
      [this] { return GetAngle() <= m_armConfig.GetMinAngle().value_or(units::degree_t{-36000}); }};
}

frc2::CommandPtr Arm::SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                            units::second_t duration) {
  auto routine = m_smc->SysId(maxVoltage, step, duration);
  return frc2::cmd::Sequence(routine.Quasistatic(frc2::sysid::Direction::kForward),
                             routine.Quasistatic(frc2::sysid::Direction::kReverse),
                             routine.Dynamic(frc2::sysid::Direction::kForward),
                             routine.Dynamic(frc2::sysid::Direction::kReverse));
}

// ---- Arm-specific interface -------------------------------------------------

frc2::CommandPtr Arm::GoToAngle(units::degree_t angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " GoToAngle");
}

frc2::CommandPtr Arm::GoToAngle(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " GoToAngle Supplier");
}

units::degree_t Arm::GetAngle() const { return m_smc->GetMechanismPosition(); }

bool Arm::IsAtAngle(units::degree_t target, units::degree_t tolerance) const {
  return std::abs(GetAngle().value() - target.value()) <= tolerance.value();
}

frc2::Trigger Arm::AtAngle(units::degree_t target, units::degree_t tolerance) {
  return frc2::Trigger{[this, target, tolerance] { return IsAtAngle(target, tolerance); }};
}

}  // namespace yams::mechanisms::positional
