// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Arm.hpp"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/time.h>

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
  if (frc::RobotBase::IsSimulation()) {
    // Configuration checks — throw descriptive exceptions like Java does.
    if (!m_armConfig->GetArmLength().has_value()) {
      throw exceptions::ArmConfigurationException(
          "Arm Length is empty", "Cannot create simulation.", "WithArmLength(units::meter_t)");
    }
    if (!m_armConfig->GetMinAngle().has_value()) {
      throw exceptions::ArmConfigurationException("Arm lower hard limit is empty",
                                                  "Cannot create simulation.",
                                                  "WithMinAngle(units::degree_t)");
    }
    if (!m_armConfig->GetMaxAngle().has_value()) {
      throw exceptions::ArmConfigurationException("Arm upper hard limit is empty",
                                                  "Cannot create simulation.",
                                                  "WithMaxAngle(units::degree_t)");
    }
    if (!m_smc->GetConfig().GetStartingPosition().has_value() &&
        !m_smc->GetConfig().GetExternalEncoderZeroOffset().has_value()) {
      throw exceptions::ArmConfigurationException("Arm starting angle is empty",
                                                  "Cannot create simulation.",
                                                  "smc.WithStartingPosition(units::degree_t)");
    }
    if (!m_smc->GetConfig().GetMOI()) {
      throw exceptions::ArmConfigurationException("Arm MOI is empty", "Cannot create simulation.",
                                                  "smc->GetConfig().WithMOI(length, mass)");
    }

    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    units::radian_t startAngle =
        m_smc->GetConfig().GetStartingPosition().value_or(units::turn_t{0});

    m_armSim.emplace(m_smc->GetDCMotor(), gearing.GetMechanismToRotorRatio(),
                     m_smc->GetConfig().GetMOI(), m_armConfig->GetArmLength().value(),
                     units::radian_t{m_armConfig->GetMinAngle().value()},
                     units::radian_t{m_armConfig->GetMaxAngle().value()}, true, startAngle,
                     std::array<double, 2>{0, 0.002 / 4096.0});

    units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::ArmSimSupplier>(
        *m_armSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, period));

    // Build Mechanism2d window.
    double armLengthM = m_armConfig->GetArmLength().value().value();
    m_mechanismWindow.emplace(armLengthM * 2.0 + 0.4, armLengthM * 2.0 + 0.4);
    m_mechanismRoot =
        m_mechanismWindow->GetRoot(m_name + "Root", armLengthM + 0.2, armLengthM + 0.2);

    units::degree_t startDeg = startAngle;
    m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        m_name, armLengthM, startDeg, 6, m_armConfig->GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        "Setpoint", armLengthM, startDeg, 3, frc::Color8Bit{frc::Color::kWhite});

    constexpr double kTickLength = 3.0 * 0.0254;  // 3 inches in metres
    m_mechanismRoot->Append<frc::MechanismLigament2d>("MaxHard", kTickLength,
                                                      m_armConfig->GetMaxAngle().value(), 4,
                                                      frc::Color8Bit{frc::Color::kLimeGreen});
    m_mechanismRoot->Append<frc::MechanismLigament2d>("MinHard", kTickLength,
                                                      m_armConfig->GetMinAngle().value(), 4,
                                                      frc::Color8Bit{frc::Color::kRed});

    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    if (smcUpperLimit.has_value() && smcLowerLimit.has_value()) {
      m_mechanismRoot->Append<frc::MechanismLigament2d>(
          "MaxSoft", kTickLength, smcUpperLimit.value(), 4, frc::Color8Bit{frc::Color::kHotPink});
      m_mechanismRoot->Append<frc::MechanismLigament2d>(
          "MinSoft", kTickLength, smcLowerLimit.value(), 4, frc::Color8Bit{frc::Color::kYellow});
    }

    frc::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
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
  if (m_setpointLigament) {
    m_setpointLigament->SetAngle(m_smc->GetMechanismPositionSetpoint().value_or(GetAngle()));
  }
}

std::string Arm::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

frc2::Trigger Arm::Max() {
  return frc2::Trigger{
      [this] { return GetAngle() >= m_armConfig->GetMaxAngle().value_or(units::degree_t{36000}); }};
}

frc2::Trigger Arm::Min() {
  return frc2::Trigger{[this] {
    return GetAngle() <= m_armConfig->GetMinAngle().value_or(units::degree_t{-36000});
  }};
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

const config::ArmConfig& Arm::GetConfig() const { return *m_armConfig; }

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
