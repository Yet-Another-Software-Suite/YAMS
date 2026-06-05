// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Arm.hpp"

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
    m_smc->SimIterate();
    ss->FeedWatchdog();

    if (m_armConfig.GetMinAngle() && ss->GetMechanismVelocity().value() < 0.0 &&
        GetAngle() < *m_armConfig.GetMinAngle()) {
      m_smc->SetEncoderPosition(*m_armConfig.GetMinAngle());
    }
    if (m_armConfig.GetMaxAngle() && ss->GetMechanismVelocity().value() > 0.0 &&
        GetAngle() > *m_armConfig.GetMaxAngle()) {
      m_smc->SetEncoderPosition(*m_armConfig.GetMaxAngle());
    }
    frc::sim::RoboRioSim::SetVInVoltage(
        frc::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    VisualizationUpdate();
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

frc2::CommandPtr Arm::Run(units::degree_t angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " Run");
}

frc2::CommandPtr Arm::Run(std::function<units::degree_t()> angle) {
  return frc2::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

frc2::CommandPtr Arm::RunTo(units::degree_t angle, units::degree_t tolerance) {
  frc2::Trigger near = IsNear(angle, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

frc2::CommandPtr Arm::RunTo(std::function<units::degree_t()> angle, units::degree_t tolerance) {
  units::degree_t target = angle();
  frc2::Trigger near = IsNear(target, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, target] { SetMechanismPositionSetpoint(target); }, {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

frc2::Trigger Arm::Gte(units::degree_t angle) {
  return frc2::Trigger{[this, angle] { return GetAngle() >= angle; }};
}

frc2::Trigger Arm::Lte(units::degree_t angle) {
  return frc2::Trigger{[this, angle] { return GetAngle() <= angle; }};
}

frc2::Trigger Arm::Between(units::degree_t start, units::degree_t end) {
  return Gte(start) && (Lte(end));
}

frc2::Trigger Arm::IsNear(units::degree_t angle, units::degree_t within) {
  return frc2::Trigger{[this, angle, within] {
    return std::abs(GetAngle().value() - angle.value()) <= within.value();
  }};
}

const config::ArmConfig& Arm::GetConfig() const { return m_armConfig; }

frc::Translation3d Arm::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return frc::Translation3d{units::meter_t{m_mechanismLigament->GetLength()},
                              frc::Rotation3d{0_rad, 0_rad, units::radian_t{GetAngle()}}};
  }
  return frc::Translation3d{};
}

void Arm::SetAngle(units::degree_t angle) { SetMechanismPositionSetpoint(angle); }

units::degree_t Arm::GetAngle() const { return m_smc->GetMechanismPosition(); }

}  // namespace yams::mechanisms::positional
