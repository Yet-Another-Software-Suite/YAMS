// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Elevator.hpp"

#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/simulation/BatterySim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/smartdashboard/Mechanism2d.hpp>
#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/smartdashboard/MechanismRoot2d.hpp>
#include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/time.hpp>

#include <array>
#include <cmath>
#include <memory>
#include <numbers>
#include <string>

#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/simulation/ElevatorSimSupplier.hpp"

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Elevator::Elevator(config::ElevatorConfig* config, motorcontrollers::SmartMotorController* smc)
    : SmartPositionalMechanism() {
  m_elevatorConfig = config;
  m_smc = smc;
  m_subsystem = m_smc->GetConfig().GetSubsystem();

  if (!m_elevatorConfig->GetTelemetryName().empty()) {
    m_name = m_elevatorConfig->GetTelemetryName();
  }

  // Apply soft limits to the motor controller when configured.
  if (auto minH = m_elevatorConfig->GetMinHeight()) {
    m_smc->SetMeasurementLowerLimit(*minH);
  }
  if (auto maxH = m_elevatorConfig->GetMaxHeight()) {
    m_smc->SetMeasurementUpperLimit(*maxH);
  }

  // Seed the encoder from the configured starting position.
  if (auto startPos = m_smc->GetConfig().GetStartingPosition()) {
    m_smc->SetEncoderPosition(m_smc->GetConfig().ConvertFromMechanism(*startPos));
  }

  if (wpi::RobotBase::IsSimulation()) {
    // Configuration checks.
    if (!m_elevatorConfig->GetCarriageMass().has_value()) {
      throw exceptions::ElevatorConfigurationException("Mass is not configured!",
                                                       "Cannot create simulator",
                                                       "WithCarriageMass(wpi::units::kilogram_t)");
    }
    if (!m_elevatorConfig->GetMinHeight().has_value()) {
      throw exceptions::ElevatorConfigurationException("Minimum height is not configured!",
                                                       "Cannot create simulator",
                                                       "WithMinimumHeight(wpi::units::meter_t)");
    }
    if (!m_elevatorConfig->GetMaxHeight().has_value()) {
      throw exceptions::ElevatorConfigurationException("Maximum height is not configured!",
                                                       "Cannot create simulator",
                                                       "WithMaximumHeight(wpi::units::meter_t)");
    }
    if (!m_smc->GetConfig().GetStartingPosition().has_value()) {
      throw exceptions::ElevatorConfigurationException("Starting position is not configured!",
                                                       "Cannot create simulator",
                                                       "smc.WithStartingPosition(wpi::units::meter_t)");
    }
    if (!m_smc->GetConfig().GetMechanismCircumference().has_value()) {
      throw exceptions::ElevatorConfigurationException(
          "Mechanism circumference is not configured!", "Cannot create simulator",
          "SMC.WithMechanismCircumference(wpi::units::meter_t)");
    }

    wpi::math::DCMotor dcMotor = m_smc->GetDCMotor();
    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    bool simulateGravity = !m_elevatorConfig->IsHorizontal();
    wpi::units::meter_t circumference = m_smc->GetConfig().GetMechanismCircumference().value();

    wpi::units::meter_t startH =
        m_smc->GetConfig().ConvertFromMechanism(*m_smc->GetConfig().GetStartingPosition());

    m_elevatorSim.emplace(
        dcMotor, gearing.GetMechanismToRotorRatio(), m_elevatorConfig->GetCarriageMass().value(),
        circumference / (2.0 * std::numbers::pi), m_elevatorConfig->GetMinHeight().value(),
        m_elevatorConfig->GetMaxHeight().value(), simulateGravity, startH,
        std::array<double, 2>{0.01 / 4096.0, 0.01 / 4096.0});

    wpi::units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::ElevatorSimSupplier>(
        *m_elevatorSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, circumference,
        period));

    // Build Mechanism2d window.
    double maxH = m_elevatorConfig->GetMaxHeight().value().value();
    double startHValue = startH.value();
    wpi::units::degree_t angle = m_elevatorConfig->GetAngle();
    constexpr double kSoftOffset = 6.0 * 0.0254;  // 6 inches
    constexpr double kHardOffset = 8.0 * 0.0254;  // 8 inches

    m_mechanismWindow.emplace(maxH * 2.0, maxH * 2.0);
    m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + "Root", maxH, 0.0);

    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    if (smcLowerLimit.has_value()) {
      m_mechanismWindow->GetRoot("MinSoft", maxH - kSoftOffset, 0.0)
          ->Append<wpi::MechanismLigament2d>(
              "Limit", m_smc->GetConfig().ConvertFromMechanism(smcLowerLimit.value()).value(),
              angle, 3, wpi::util::Color8Bit{wpi::util::Color::YELLOW});
    }
    if (smcUpperLimit.has_value()) {
      m_mechanismWindow->GetRoot("MaxSoft", maxH - kSoftOffset, 0.0)
          ->Append<wpi::MechanismLigament2d>(
              "Limit", m_smc->GetConfig().ConvertFromMechanism(smcUpperLimit.value()).value(),
              angle, 3, wpi::util::Color8Bit{wpi::util::Color::HOT_PINK});
    }
    m_mechanismWindow->GetRoot("MinHard", maxH - kHardOffset, 0.0)
        ->Append<wpi::MechanismLigament2d>("Limit",
                                           m_elevatorConfig->GetMinHeight().value().value(), angle,
                                           3, wpi::util::Color8Bit{wpi::util::Color::RED});
    m_mechanismWindow->GetRoot("MaxHard", maxH - kHardOffset, 0.0)
        ->Append<wpi::MechanismLigament2d>("Limit",
                                           m_elevatorConfig->GetMaxHeight().value().value(), angle,
                                           3, wpi::util::Color8Bit{wpi::util::Color::LIME_GREEN});

    m_mechanismLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        m_name, startHValue, angle, 6, m_elevatorConfig->GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        "Setpoint", startHValue, angle, 3, wpi::util::Color8Bit{wpi::util::Color::WHITE});

    wpi::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
  }
}

// ---- SmartMechanism overrides -----------------------------------------------

void Elevator::SimIterate() {
  if (m_elevatorSim.has_value() && m_smc->GetSimSupplier()) {
    auto* ss = m_smc->GetSimSupplier();
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->StarveWatchdog();

    if (!m_elevatorConfig->GetMinHeight() || GetHeight() >= *m_elevatorConfig->GetMinHeight()) {
      wpi::sim::RoboRioSim::SetVInVoltage(
          wpi::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
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

wpi::cmd::Trigger Elevator::Max() {
  return wpi::cmd::Trigger{[this] {
    return GetHeight() >= m_elevatorConfig->GetMaxHeight().value_or(wpi::units::meter_t{99});
  }};
}

wpi::cmd::Trigger Elevator::Min() {
  return wpi::cmd::Trigger{[this] {
    return GetHeight() <= m_elevatorConfig->GetMinHeight().value_or(wpi::units::meter_t{-99});
  }};
}

// ---- Elevator-specific interface --------------------------------------------

wpi::cmd::CommandPtr Elevator::Run(wpi::units::meter_t height) {
  return wpi::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height); }, {m_subsystem})
      .WithName(m_name + " Run");
}

wpi::cmd::CommandPtr Elevator::Run(std::function<wpi::units::meter_t()> height) {
  return wpi::cmd::Run([this, height] { SetMeasurementPositionSetpoint(height()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

wpi::cmd::CommandPtr Elevator::RunTo(wpi::units::meter_t height, wpi::units::meter_t tolerance) {
  wpi::cmd::Trigger near = IsNear(height, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, height] { SetMeasurementPositionSetpoint(height); },
                            {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

wpi::cmd::CommandPtr Elevator::RunTo(std::function<wpi::units::meter_t()> height, wpi::units::meter_t tolerance) {
  wpi::units::meter_t target = height();
  wpi::cmd::Trigger near = IsNear(target, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, target] { SetMeasurementPositionSetpoint(target); },
                            {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

wpi::cmd::Trigger Elevator::Gte(wpi::units::meter_t height) {
  return wpi::cmd::Trigger{[this, height] { return GetHeight() >= height; }};
}

wpi::cmd::Trigger Elevator::Lte(wpi::units::meter_t height) {
  return wpi::cmd::Trigger{[this, height] { return GetHeight() <= height; }};
}

wpi::cmd::Trigger Elevator::Between(wpi::units::meter_t start, wpi::units::meter_t end) {
  return Gte(start) && (Lte(end));
}

wpi::cmd::Trigger Elevator::IsNear(wpi::units::meter_t height, wpi::units::meter_t within) {
  return wpi::cmd::Trigger{[this, height, within] {
    return std::abs(GetHeight().value() - height.value()) <= within.value();
  }};
}

const config::ElevatorConfig& Elevator::GetConfig() const { return *m_elevatorConfig; }

wpi::math::Translation3d Elevator::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return wpi::math::Translation3d{0_m, 0_m, wpi::units::meter_t{m_mechanismLigament->GetLength()}};
  }
  return wpi::math::Translation3d{};
}

void Elevator::SetHeight(wpi::units::meter_t height) { SetMeasurementPositionSetpoint(height); }

wpi::units::meter_t Elevator::GetHeight() const { return m_smc->GetMeasurementPosition(); }

}  // namespace yams::mechanisms::positional
