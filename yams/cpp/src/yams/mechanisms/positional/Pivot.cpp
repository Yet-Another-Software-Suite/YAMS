// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Pivot.hpp"

#include <frc/RobotBase.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/system/plant/LinearSystemId.h>
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
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

namespace yams::mechanisms::positional {

// ---- Constructor ------------------------------------------------------------

Pivot::Pivot(config::PivotConfig* config, motorcontrollers::SmartMotorController* smc)
    : SmartPositionalMechanism() {
  m_pivotConfig = config;
  m_smc = smc;
  m_subsystem = m_smc->GetConfig().GetSubsystem();

  if (!m_pivotConfig->GetTelemetryName().empty()) {
    m_name = m_pivotConfig->GetTelemetryName();
  }

  // Apply angular soft limits to the motor controller when configured.
  if (auto minA = m_pivotConfig->GetMinAngle()) {
    m_smc->SetMechanismLowerLimit(*minA);
  }
  if (auto maxA = m_pivotConfig->GetMaxAngle()) {
    m_smc->SetMechanismUpperLimit(*maxA);
  }

  // Seed the encoder from the configured starting position.
  if (auto startA = m_smc->GetConfig().GetStartingPosition()) {
    m_smc->SetEncoderPosition(*startA);
  }

  if (frc::RobotBase::IsSimulation()) {
    // Configuration checks.
    if (!m_pivotConfig->GetMinAngle().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot lower hard limit is empty",
                                                    "Cannot create simulation.",
                                                    "WithMinAngle(units::degree_t)");
    }
    if (!m_pivotConfig->GetMaxAngle().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot upper hard limit is empty",
                                                    "Cannot create simulation.",
                                                    "WithMaxAngle(units::degree_t)");
    }
    if (!m_smc->GetConfig().GetStartingPosition().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot starting angle is empty",
                                                    "Cannot create simulation.",
                                                    "smc.WithStartingPosition(units::degree_t)");
    }
    if (!m_smc->GetConfig().GetMOI()) {
      throw exceptions::PivotConfigurationException("Pivot MOI is empty",
                                                    "Cannot create simulation.",
                                                    "smc->GetConfig().WithMOI(length, mass)");
    }

    // Create DCMotorSim and wire up DCMotorSimSupplier.
    frc::DCMotor dcMotor = m_smc->GetDCMotor();
    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    auto plant = frc::LinearSystemId::DCMotorSystem(dcMotor, m_smc->GetConfig().GetMOI(),
                                                    gearing.GetMechanismToRotorRatio());
    m_dcMotorSim.emplace(plant, dcMotor);

    units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::DCMotorSimSupplier>(
        *m_dcMotorSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, period));

    // Build Mechanism2d — fixed 36-inch ligament length like Java.
    constexpr double kPivotLen = 36.0 * 0.0254;  // 36 inches in metres
    m_mechanismWindow.emplace(kPivotLen * 2.0, kPivotLen * 2.0);
    m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + "Root", kPivotLen, kPivotLen);

    units::degree_t startDeg = *m_smc->GetConfig().GetStartingPosition();
    m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        m_name, kPivotLen, startDeg, 6, m_pivotConfig->GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
        "Setpoint", kPivotLen, startDeg, 3, frc::Color8Bit{frc::Color::kWhite});

    constexpr double kTickLen = 3.0 * 0.0254;  // 3 inches in metres
    m_mechanismRoot->Append<frc::MechanismLigament2d>("MaxHard", kTickLen,
                                                      m_pivotConfig->GetMaxAngle().value(), 4,
                                                      frc::Color8Bit{frc::Color::kLimeGreen});
    m_mechanismRoot->Append<frc::MechanismLigament2d>("MinHard", kTickLen,
                                                      m_pivotConfig->GetMinAngle().value(), 4,
                                                      frc::Color8Bit{frc::Color::kRed});

    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    if (smcUpperLimit.has_value() && smcLowerLimit.has_value()) {
      m_mechanismRoot->Append<frc::MechanismLigament2d>("MaxSoft", kTickLen, smcUpperLimit.value(),
                                                        4, frc::Color8Bit{frc::Color::kHotPink});
      m_mechanismRoot->Append<frc::MechanismLigament2d>("MinSoft", kTickLen, smcLowerLimit.value(),
                                                        4, frc::Color8Bit{frc::Color::kYellow});
    }

    frc::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
  }
}

// ---- SmartMechanism overrides -----------------------------------------------

void Pivot::SimIterate() {
  if (m_dcMotorSim.has_value() && m_smc->GetSimSupplier()) {
    auto* ss = m_smc->GetSimSupplier();
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->StarveWatchdog();

    double simVelRadPerSec = m_dcMotorSim->GetAngularVelocity().value();
    if (m_pivotConfig->GetMinAngle() && simVelRadPerSec < 0.0 &&
        GetAngle() < *m_pivotConfig->GetMinAngle()) {
      m_smc->SetEncoderPosition(*m_pivotConfig->GetMinAngle());
    }
    if (m_pivotConfig->GetMaxAngle() && simVelRadPerSec > 0.0 &&
        GetAngle() > *m_pivotConfig->GetMaxAngle()) {
      m_smc->SetEncoderPosition(*m_pivotConfig->GetMaxAngle());
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
  if (m_setpointLigament) {
    m_setpointLigament->SetAngle(m_smc->GetMechanismPositionSetpoint().value_or(GetAngle()));
  }
}

std::string Pivot::GetName() const { return m_name; }

// ---- SmartPositionalMechanism overrides -------------------------------------

frc2::Trigger Pivot::Max() {
  return frc2::Trigger{[this] {
    return GetAngle() >= m_pivotConfig->GetMaxAngle().value_or(units::degree_t{36000});
  }};
}

frc2::Trigger Pivot::Min() {
  return frc2::Trigger{[this] {
    return GetAngle() <= m_pivotConfig->GetMinAngle().value_or(units::degree_t{-36000});
  }};
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

const config::PivotConfig& Pivot::GetConfig() const { return *m_pivotConfig; }

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
