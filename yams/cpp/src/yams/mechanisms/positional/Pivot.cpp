// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/Pivot.hpp"

#include <wpi/framework/RobotBase.hpp>
#include <wpi/math/geometry/Rotation3d.hpp>
#include <wpi/math/geometry/Translation3d.hpp>
#include <wpi/simulation/BatterySim.hpp>
#include <wpi/simulation/RoboRioSim.hpp>
#include <wpi/smartdashboard/Mechanism2d.hpp>
#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/smartdashboard/MechanismRoot2d.hpp>
#include <wpi/smartdashboard/SmartDashboard.hpp>
#include <wpi/math/system/Models.hpp>
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

  if (wpi::RobotBase::IsSimulation()) {
    // Configuration checks.
    if (!m_pivotConfig->GetMinAngle().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot lower hard limit is empty",
                                                    "Cannot create simulation.",
                                                    "WithMinAngle(wpi::units::degree_t)");
    }
    if (!m_pivotConfig->GetMaxAngle().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot upper hard limit is empty",
                                                    "Cannot create simulation.",
                                                    "WithMaxAngle(wpi::units::degree_t)");
    }
    if (!m_smc->GetConfig().GetStartingPosition().has_value()) {
      throw exceptions::PivotConfigurationException("Pivot starting angle is empty",
                                                    "Cannot create simulation.",
                                                    "smc.WithStartingPosition(wpi::units::degree_t)");
    }
    if (!m_smc->GetConfig().GetMOI()) {
      throw exceptions::PivotConfigurationException("Pivot MOI is empty",
                                                    "Cannot create simulation.",
                                                    "smc->GetConfig().WithMOI(length, mass)");
    }

    // Create DCMotorSim and wire up DCMotorSimSupplier.
    wpi::math::DCMotor dcMotor = m_smc->GetDCMotor();
    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(dcMotor, m_smc->GetConfig().GetMOI(),
                                                    gearing.GetMechanismToRotorRatio());
    m_dcMotorSim.emplace(plant, dcMotor);

    wpi::units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::DCMotorSimSupplier>(
        *m_dcMotorSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, period));

    // Build Mechanism2d — fixed 36-inch ligament length like Java.
    constexpr double kPivotLen = 36.0 * 0.0254;  // 36 inches in metres
    m_mechanismWindow.emplace(kPivotLen * 2.0, kPivotLen * 2.0);
    m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + "Root", kPivotLen, kPivotLen);

    wpi::units::degree_t startDeg = *m_smc->GetConfig().GetStartingPosition();
    m_mechanismLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        m_name, kPivotLen, startDeg, 6, m_pivotConfig->GetSimColor());
    m_setpointLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        "Setpoint", kPivotLen, startDeg, 3, wpi::util::Color8Bit{wpi::util::Color::WHITE});

    constexpr double kTickLen = 3.0 * 0.0254;  // 3 inches in metres
    m_mechanismRoot->Append<wpi::MechanismLigament2d>("MaxHard", kTickLen,
                                                      m_pivotConfig->GetMaxAngle().value(), 4,
                                                      wpi::util::Color8Bit{wpi::util::Color::LIME_GREEN});
    m_mechanismRoot->Append<wpi::MechanismLigament2d>("MinHard", kTickLen,
                                                      m_pivotConfig->GetMinAngle().value(), 4,
                                                      wpi::util::Color8Bit{wpi::util::Color::RED});

    auto smcUpperLimit = m_smc->GetConfig().GetMechanismUpperLimit();
    auto smcLowerLimit = m_smc->GetConfig().GetMechanismLowerLimit();
    if (smcUpperLimit.has_value() && smcLowerLimit.has_value()) {
      m_mechanismRoot->Append<wpi::MechanismLigament2d>("MaxSoft", kTickLen, smcUpperLimit.value(),
                                                        4, wpi::util::Color8Bit{wpi::util::Color::HOT_PINK});
      m_mechanismRoot->Append<wpi::MechanismLigament2d>("MinSoft", kTickLen, smcLowerLimit.value(),
                                                        4, wpi::util::Color8Bit{wpi::util::Color::YELLOW});
    }

    wpi::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
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
    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
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

wpi::cmd::Trigger Pivot::Max() {
  return wpi::cmd::Trigger{[this] {
    return GetAngle() >= m_pivotConfig->GetMaxAngle().value_or(wpi::units::degree_t{36000});
  }};
}

wpi::cmd::Trigger Pivot::Min() {
  return wpi::cmd::Trigger{[this] {
    return GetAngle() <= m_pivotConfig->GetMinAngle().value_or(wpi::units::degree_t{-36000});
  }};
}

// ---- Pivot-specific interface -----------------------------------------------

wpi::cmd::CommandPtr Pivot::Run(wpi::units::degree_t angle) {
  return wpi::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .WithName(m_name + " Run");
}

wpi::cmd::CommandPtr Pivot::Run(std::function<wpi::units::degree_t()> angle) {
  return wpi::cmd::Run([this, angle] { SetMechanismPositionSetpoint(angle()); }, {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

wpi::cmd::CommandPtr Pivot::RunTo(wpi::units::degree_t angle, wpi::units::degree_t tolerance) {
  wpi::cmd::Trigger near = IsNear(angle, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, angle] { SetMechanismPositionSetpoint(angle); }, {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

wpi::cmd::CommandPtr Pivot::RunTo(std::function<wpi::units::degree_t()> angle, wpi::units::degree_t tolerance) {
  wpi::units::degree_t target = angle();
  wpi::cmd::Trigger near = IsNear(target, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce([this, target] { SetMechanismPositionSetpoint(target); }, {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

wpi::cmd::Trigger Pivot::Gte(wpi::units::degree_t angle) {
  return wpi::cmd::Trigger{[this, angle] { return GetAngle() >= angle; }};
}

wpi::cmd::Trigger Pivot::Lte(wpi::units::degree_t angle) {
  return wpi::cmd::Trigger{[this, angle] { return GetAngle() <= angle; }};
}

wpi::cmd::Trigger Pivot::Between(wpi::units::degree_t start, wpi::units::degree_t end) {
  return Gte(start) && (Lte(end));
}

wpi::cmd::Trigger Pivot::IsNear(wpi::units::degree_t angle, wpi::units::degree_t within) {
  return wpi::cmd::Trigger{[this, angle, within] {
    return std::abs(GetAngle().value() - angle.value()) <= within.value();
  }};
}

const config::PivotConfig& Pivot::GetConfig() const { return *m_pivotConfig; }

wpi::math::Translation3d Pivot::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return wpi::math::Translation3d{wpi::units::meter_t{m_mechanismLigament->GetLength()},
                              wpi::math::Rotation3d{0_rad, 0_rad, wpi::units::radian_t{GetAngle()}}};
  }
  return wpi::math::Translation3d{};
}

void Pivot::SetAngle(wpi::units::degree_t angle) { SetMechanismPositionSetpoint(angle); }

wpi::units::degree_t Pivot::GetAngle() const { return m_smc->GetMechanismPosition(); }

}  // namespace yams::mechanisms::positional
