// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Arm.hpp"

#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/simulation/BatterySim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/smartdashboard/Mechanism2d.hpp>
#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/smartdashboard/MechanismRoot2d.hpp>
#include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/math/system/DCMotor.hpp>
#include <wpi/util/Color.hpp>
#include <wpi/util/Color8Bit.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/time.hpp>

#include <cmath>
#include <memory>
#include <string>

#include "yams/exceptions.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/simulation/ArmSimSupplier.hpp"

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Arm::Arm(config::ArmConfig* config, motorcontrollers::SmartMotorController* smc)
    : SmartPositionalMechanism() {
  m_armConfig = config;
  m_smc = smc;
  m_subsystem = m_smc->GetConfig().GetSubsystem();

  if (!m_armConfig->GetTelemetryName().empty()) {
    m_name = m_armConfig->GetTelemetryName();
  }

  // Apply angular soft limits to the motor controller when configured.
  if (auto minA = m_armConfig->GetMinAngle()) {
    m_smc->SetMechanismLowerLimit(*minA);
  }
  if (auto maxA = m_armConfig->GetMaxAngle()) {
    m_smc->SetMechanismUpperLimit(*maxA);
  }

  // Seed the encoder from the configured starting position.
  if (auto startA = m_smc->GetConfig().GetStartingPosition()) {
    m_smc->SetEncoderPosition(*startA);
  }
  if (wpi::RobotBase::IsSimulation()) {
    // Configuration checks — throw descriptive exceptions like Java does.
    if (!m_armConfig->GetArmLength().has_value()) {
      throw exceptions::ArmConfigurationException(
          "Arm Length is empty", "Cannot create simulation.", "WithArmLength(wpi::units::meter_t)");
    }
    if (!m_armConfig->GetMinAngle().has_value()) {
      throw exceptions::ArmConfigurationException("Arm lower hard limit is empty",
                                                  "Cannot create simulation.",
                                                  "WithMinAngle(wpi::units::degree_t)");
    }
    if (!m_armConfig->GetMaxAngle().has_value()) {
      throw exceptions::ArmConfigurationException("Arm upper hard limit is empty",
                                                  "Cannot create simulation.",
                                                  "WithMaxAngle(wpi::units::degree_t)");
    }
    if (!m_smc->GetConfig().GetStartingPosition().has_value() &&
        !m_smc->GetConfig().GetExternalEncoderZeroOffset().has_value()) {
      throw exceptions::ArmConfigurationException("Arm starting angle is empty",
                                                  "Cannot create simulation.",
                                                  "smc.WithStartingPosition(wpi::units::degree_t)");
    }
    if (!m_smc->GetConfig().GetMOI()) {
      throw exceptions::ArmConfigurationException("Arm MOI is empty", "Cannot create simulation.",
                                                  "smc->GetConfig().WithMOI(length, mass)");
    }

    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    wpi::units::radian_t startAngle =
        m_smc->GetConfig().GetStartingPosition().value_or(wpi::units::turn_t{0});

    m_armSim.emplace(m_smc->GetDCMotor(), gearing.GetMechanismToRotorRatio(),
                     m_smc->GetConfig().GetMOI(), m_armConfig->GetArmLength().value(),
                     wpi::units::radian_t{m_armConfig->GetMinAngle().value()},
                     wpi::units::radian_t{m_armConfig->GetMaxAngle().value()}, true, startAngle,
                     std::array<double, 2>{0, 0.002 / 4096.0});

    wpi::units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::ArmSimSupplier>(
        *m_armSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, period));

    // Build Mechanism2d window.
    double armLengthM = m_armConfig->GetArmLength().value().value();
    m_mechanismWindow.emplace(armLengthM * 2.0 + 0.4, armLengthM * 2.0 + 0.4);
    m_mechanismRoot =
        m_mechanismWindow->GetRoot(m_name + "Root", armLengthM + 0.2, armLengthM + 0.2);

    wpi::units::degree_t startDeg = startAngle;
    m_mechanismLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        m_name, armLengthM, startDeg, 6, m_armConfig->GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        "Setpoint", armLengthM, startDeg, 3, wpi::util::Color8Bit{wpi::util::Color::WHITE});

    constexpr double kTickLength = 3.0 * 0.0254;  // 3 inches in metres
    m_mechanismRoot->Append<wpi::MechanismLigament2d>("MaxHard", kTickLength,
                                                      m_armConfig->GetMaxAngle().value(), 4,
                                                      wpi::util::Color8Bit{wpi::util::Color::LIME_GREEN});
    m_mechanismRoot->Append<wpi::MechanismLigament2d>("MinHard", kTickLength,
                                                      m_armConfig->GetMinAngle().value(), 4,
                                                      wpi::util::Color8Bit{wpi::util::Color::RED});

    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    if (smcUpperLimit.has_value() && smcLowerLimit.has_value()) {
      m_mechanismRoot->Append<wpi::MechanismLigament2d>(
          "MaxSoft", kTickLength, smcUpperLimit.value(), 4, wpi::util::Color8Bit{wpi::util::Color::HOT_PINK});
      m_mechanismRoot->Append<wpi::MechanismLigament2d>(
          "MinSoft", kTickLength, smcLowerLimit.value(), 4, wpi::util::Color8Bit{wpi::util::Color::YELLOW});
    }

    wpi::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
  }
}

// ---- SmartMechanism overrides -----------------------------------------------

void Arm::SimIterate() {
  if (m_armSim.has_value() && m_smc->GetSimSupplier()) {
    auto* ss = m_smc->GetSimSupplier();
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->StarveWatchdog();

    if (m_armConfig->GetMinAngle() && m_armSim->GetVelocity().value() < 0.0 &&
        GetAngle() < *m_armConfig->GetMinAngle()) {
      m_smc->SetEncoderPosition(*m_armConfig->GetMinAngle());
    }
    if (m_armConfig->GetMaxAngle() && m_armSim->GetVelocity().value() > 0.0 &&
        GetAngle() > *m_armConfig->GetMaxAngle()) {
      m_smc->SetEncoderPosition(*m_armConfig->GetMaxAngle());
    }
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    VisualizationUpdate();
  }
}

void Arm::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void Arm::VisualizationUpdate() {
  if (m_mechanismLigament) {
    m_mechanismLigament->SetAngle(GetAngle());
  }
  if (m_setpointLigament) {
    m_setpointLigament->SetAngle(m_smc->GetMechanismPositionSetpoint().value_or(GetAngle()));
  }
}

std::string Arm::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

wpi::cmd::Trigger Arm::Max() {
  return wpi::cmd::Trigger{
      [this] { return GetAngle() >= m_armConfig->GetMaxAngle().value_or(wpi::units::degree_t{36000}); }};
}

wpi::cmd::Trigger Arm::Min() {
  return wpi::cmd::Trigger{[this] {
    return GetAngle() <= m_armConfig->GetMinAngle().value_or(wpi::units::degree_t{-36000});
  }};
}

// ---- Arm-specific interface -------------------------------------------------

wpi::cmd::CommandPtr Arm::Run(wpi::units::degree_t angle) {
  return wpi::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " Run");
}

wpi::cmd::CommandPtr Arm::Run(std::function<wpi::units::degree_t()> angle) {
  return wpi::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

wpi::cmd::CommandPtr Arm::RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance) {
  wpi::cmd::Trigger near = IsNear(angle, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

wpi::cmd::CommandPtr Arm::RunTo(std::function<wpi::units::degree_t()> angle, wpi::units::degree_t tolerance) {
  wpi::units::degree_t target = angle();
  wpi::cmd::Trigger near = IsNear(target, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, target] { SetMechanismPositionSetpoint(target); }, {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

wpi::cmd::Trigger Arm::Gte(wpi::units::degree_t angle) {
  return wpi::cmd::Trigger{[this, angle] { return GetAngle() >= angle; }};
}

wpi::cmd::Trigger Arm::Lte(wpi::units::degree_t angle) {
  return wpi::cmd::Trigger{[this, angle] { return GetAngle() <= angle; }};
}

wpi::cmd::Trigger Arm::Between(wpi::units::degree_t start, wpi::units::degree_t end) {
  return Gte(start) && (Lte(end));
}

wpi::cmd::Trigger Arm::IsNear(wpi::units::degree_t angle, wpi::units::degree_t within) {
  return wpi::cmd::Trigger{[this, angle, within] {
    return std::abs(GetAngle().value() - angle.value()) <= within.value();
  }};
}

const config::ArmConfig& Arm::GetConfig() const { return *m_armConfig; }

wpi::math::Translation3d Arm::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return wpi::math::Translation3d{wpi::units::meter_t{m_mechanismLigament->GetLength()},
                              wpi::math::Rotation3d{0_rad, 0_rad, wpi::units::radian_t{GetAngle()}}};
  }
  return wpi::math::Translation3d{};
}

void Arm::SetAngle(wpi::units::degree_t angle) { SetMechanismPositionSetpoint(angle); }

wpi::units::degree_t Arm::GetAngle() const { return m_smc->GetMechanismPosition(); }

}  // namespace yams::mechanisms::positional
