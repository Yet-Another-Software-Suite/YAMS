// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Elevator.hpp"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/Commands.h>
#include <units/length.h>
#include <units/time.h>

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include "yams/exceptions/ElevatorConfigurationException.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

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

  if (frc::RobotBase::IsSimulation()) {
    // Configuration checks.
    if (!config.GetCarriageMass().has_value()) {
      throw ElevatorConfigurationException("Mass is not configured!", "Cannot create simulator",
                                           "WithCarriageMass(units::kilogram_t)");
    }
    if (!config.GetMinHeight().has_value()) {
      throw ElevatorConfigurationException("Minimum height is not configured!",
                                           "Cannot create simulator",
                                           "WithMinimumHeight(units::meter_t)");
    }
    if (!config.GetMaxHeight().has_value()) {
      throw ElevatorConfigurationException("Maximum height is not configured!",
                                           "Cannot create simulator",
                                           "WithMaximumHeight(units::meter_t)");
    }
    if (!config.GetStartingHeight().has_value()) {
      throw ElevatorConfigurationException("Starting height is not configured!",
                                           "Cannot create simulator",
                                           "WithStartingHeight(units::meter_t)");
    }
    if (!m_smc->GetConfig().GetMechanismCircumference().has_value()) {
      throw ElevatorConfigurationException("Mechanism circumference is not configured!",
                                           "Cannot create simulator",
                                           "SMC.WithMechanismCircumference(units::meter_t)");
    }

    m_smc->SetupSimulation();

    frc::DCMotor dcMotor = m_smc->GetDCMotor();
    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    bool simulateGravity = !config.IsHorizontal();
    units::meter_t circumference = m_smc->GetConfig().GetMechanismCircumference().value();

    m_elevatorSim.emplace(dcMotor, gearing.GetMechanismToRotorRatio(),
                          config.GetCarriageMass().value(), circumference / (2.0 * std::numbers::pi),
                          config.GetMinHeight().value(), config.GetMaxHeight().value(),
                          simulateGravity, config.GetStartingHeight().value(),
                          std::array<double, 2>{0.01 / 4096.0, 0.01 / 4096.0});

    units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::ElevatorSimSupplier>(
        *m_elevatorSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, circumference,
        period));

    // Build Mechanism2d window.
    double maxH = config.GetMaxHeight().value().value();
    double startH = config.GetStartingHeight().value().value();
    units::degree_t angle = config.GetAngle();
    constexpr double kSoftOffset = 6.0 * 0.0254;  // 6 inches
    constexpr double kHardOffset = 8.0 * 0.0254;  // 8 inches

    m_mechanismWindow.emplace(maxH * 2.0, maxH * 2.0);
    m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + "Root", maxH, 0.0);

    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    if (smcLowerLimit.has_value()) {
      m_mechanismWindow->GetRoot("MinSoft", maxH - kSoftOffset, 0.0)
          ->Append<frc::MechanismLigament2d>(
              "Limit", m_smc->GetConfig().ConvertFromMechanism(smcLowerLimit.value()).value(),
              angle, 3, frc::Color8Bit{frc::Color::kYellow});
    }
    if (smcUpperLimit.has_value()) {
      m_mechanismWindow->GetRoot("MaxSoft", maxH - kSoftOffset, 0.0)
          ->Append<frc::MechanismLigament2d>(
              "Limit", m_smc->GetConfig().ConvertFromMechanism(smcUpperLimit.value()).value(),
              angle, 3, frc::Color8Bit{frc::Color::kHotPink});
    }
    m_mechanismWindow->GetRoot("MinHard", maxH - kHardOffset, 0.0)
        ->Append<frc::MechanismLigament2d>("Limit", config.GetMinHeight().value().value(), angle, 3,
                                           frc::Color8Bit{frc::Color::kRed});
    m_mechanismWindow->GetRoot("MaxHard", maxH - kHardOffset, 0.0)
        ->Append<frc::MechanismLigament2d>("Limit", config.GetMaxHeight().value().value(), angle, 3,
                                           frc::Color8Bit{frc::Color::kLimeGreen});

    m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        m_name, startH, angle, 6, config.GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        "Setpoint", startH, angle, 3, frc::Color8Bit{frc::Color::kWhite});

    frc::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
  }
}

// ---- SmartMechanism overrides -----------------------------------------------

void Elevator::SimIterate() {
  if (m_elevatorSim.has_value() && m_smc->GetSimSupplier()) {
    auto* ss = m_smc->GetSimSupplier();
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->StarveWatchdog();

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
  if (m_setpointLigament) {
    auto setpoint = m_smc->GetMechanismPositionSetpoint();
    if (setpoint.has_value()) {
      m_setpointLigament->SetLength(
          m_smc->GetConfig().ConvertFromMechanism(setpoint.value()).value());
    }
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
  return Gte(start) && (Lte(end));
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
