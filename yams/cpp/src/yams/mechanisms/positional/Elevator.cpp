// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Elevator.hpp"

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
#include <units/length.h>

#include <cmath>
#include <string>

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Elevator::Elevator(const config::ElevatorConfig& config)
    : SmartPositionalMechanism(), m_elevatorConfig{config} {
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

void Elevator::SimIterate() {
  if (auto* ss = m_smc->GetSimSupplier()) {
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->FeedWatchdog();

    if (!m_elevatorConfig.GetMinHeight() || GetHeight() >= *m_elevatorConfig.GetMinHeight()) {
      frc::sim::RoboRioSim::SetVInVoltage(
          frc::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    }
    VisualizationUpdate();
  }
}

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

frc2::CommandPtr Elevator::Run(units::meter_t height) {
  return frc2::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height); }, {m_subsystem})
      .WithName(m_name + " Run");
}

frc2::CommandPtr Elevator::Run(std::function<units::meter_t()> height) {
  return frc2::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

frc2::CommandPtr Elevator::RunTo(units::meter_t height, units::meter_t tolerance) {
  frc2::Trigger near = IsNear(height, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, height] { SetMeasurementPositionSetpoint(height); },
                            {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

frc2::CommandPtr Elevator::RunTo(std::function<units::meter_t()> height, units::meter_t tolerance) {
  units::meter_t target = height();
  frc2::Trigger near = IsNear(target, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce([this, target] { SetMeasurementPositionSetpoint(target); },
                            {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

frc2::Trigger Elevator::Gte(units::meter_t height) {
  return frc2::Trigger{[this, height] { return GetHeight() >= height; }};
}

frc2::Trigger Elevator::Lte(units::meter_t height) {
  return frc2::Trigger{[this, height] { return GetHeight() <= height; }};
}

frc2::Trigger Elevator::Between(units::meter_t start, units::meter_t end) {
  return Gte(start).And(Lte(end));
}

frc2::Trigger Elevator::IsNear(units::meter_t height, units::meter_t within) {
  return frc2::Trigger{[this, height, within] {
    return std::abs(GetHeight().value() - height.value()) <= within.value();
  }};
}

const config::ElevatorConfig& Elevator::GetConfig() const { return m_elevatorConfig; }

frc::Translation3d Elevator::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return frc::Translation3d{0_m, 0_m, units::meter_t{m_mechanismLigament->GetLength()}};
  }
  return frc::Translation3d{};
}

void Elevator::SetHeight(units::meter_t height) { SetMeasurementPositionSetpoint(height); }

units::meter_t Elevator::GetHeight() const { return m_smc->GetMeasurementPosition(); }

}  // namespace yams::mechanisms::positional
