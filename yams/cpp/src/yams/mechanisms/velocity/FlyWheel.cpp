// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/velocity/FlyWheel.hpp"

#include <frc/DriverStation.h>
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
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/math.h>

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

namespace yams::mechanisms::velocity {

// ---- Constructor ------------------------------------------------------------

FlyWheel::FlyWheel(const config::FlyWheelConfig& config)
    : SmartVelocityMechanism(), m_flyWheelConfig{config} {
  m_smc = config.GetMotorController();
  m_subsystem = config.GetSubsystem();

  if (!config.GetTelemetryName().empty()) {
    m_name = config.GetTelemetryName();
  }

  // Build a Mechanism2d window — a spinning disc representation.
  constexpr double kRadius = 0.3;
  m_mechanismWindow.emplace(kRadius * 2.0 + 0.2, kRadius * 2.0 + 0.2);
  m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + " Root", kRadius + 0.1, kRadius + 0.1);
  m_mechanismLigament = m_mechanismRoot->Append<frc::MechanismLigament2d>(
      m_name + " Spoke", kRadius, 0_deg, 6, frc::Color8Bit{frc::Color::kYellow});

  frc::SmartDashboard::PutData(m_name, &(*m_mechanismWindow));
}

// ---- SmartMechanism overrides -----------------------------------------------

void FlyWheel::SimIterate() {
  if (auto* ss = m_smc->GetSimSupplier()) {
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->FeedWatchdog();

    frc::sim::RoboRioSim::SetVInVoltage(
        frc::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    VisualizationUpdate();
  }
}

void FlyWheel::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void FlyWheel::VisualizationUpdate() {
  // Rotate the spoke angle proportionally to visualise spinning.
  if (m_mechanismLigament) {
    // Accumulate angle based on current velocity (visual only — not precise).
    units::degree_t currentAngle{m_mechanismLigament->GetAngle()};
    // Advance by a fixed small step in the direction of current velocity.
    double sign = (GetVelocity().value() >= 0.0) ? 1.0 : -1.0;
    m_mechanismLigament->SetAngle(currentAngle + units::degree_t{sign * 5.0});
  }
}

std::string FlyWheel::GetName() const { return m_name; }

// ---- SmartVelocityMechanism overrides ---------------------------------------

frc2::Trigger FlyWheel::Max() {
  // FlyWheel has no inherent max velocity limit by default; always false.
  return frc2::Trigger{[] { return false; }};
}

frc2::Trigger FlyWheel::Min() {
  // FlyWheel has no inherent min velocity limit by default; always false.
  return frc2::Trigger{[] { return false; }};
}

frc2::CommandPtr FlyWheel::SysId(units::volt_t maxVoltage, frc2::sysid::ramp_rate_t step,
                                 units::second_t duration) {
  auto routine = m_smc->SysId(maxVoltage, step, duration);
  return frc2::cmd::Sequence(routine.Quasistatic(frc2::sysid::Direction::kForward),
                             routine.Quasistatic(frc2::sysid::Direction::kReverse),
                             routine.Dynamic(frc2::sysid::Direction::kForward),
                             routine.Dynamic(frc2::sysid::Direction::kReverse));
}

// ---- Run / RunTo / comparison / setpoint interface --------------------------

frc2::CommandPtr FlyWheel::Run(units::degrees_per_second_t velocity) {
  return frc2::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity); }, {m_subsystem})
      .WithName(m_name + " Run");
}

frc2::CommandPtr FlyWheel::Run(std::function<units::degrees_per_second_t()> velocity) {
  return frc2::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity()); },
                        {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

frc2::CommandPtr FlyWheel::Run(units::meters_per_second_t surfaceSpeed) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s Run: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return frc2::cmd::None();
  }
  units::meter_t radius = *diameter / 2.0;
  return frc2::cmd::Run(
             [this, surfaceSpeed, radius] {
               SetMechanismVelocitySetpoint(units::degrees_per_second_t{
                   (surfaceSpeed.value() / radius.value()) * (180.0 / std::numbers::pi)});
             },
             {m_subsystem})
      .WithName(m_name + " Run Surface");
}

frc2::CommandPtr FlyWheel::Run(std::function<units::meters_per_second_t()> surfaceSpeed) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s Run: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return frc2::cmd::None();
  }
  units::meter_t radius = *diameter / 2.0;
  return frc2::cmd::Run(
             [this, surfaceSpeed, radius] {
               SetMechanismVelocitySetpoint(units::degrees_per_second_t{
                   (surfaceSpeed().value() / radius.value()) * (180.0 / std::numbers::pi)});
             },
             {m_subsystem})
      .WithName(m_name + " Run Surface Supplier");
}

frc2::CommandPtr FlyWheel::RunTo(units::degrees_per_second_t velocity,
                                 units::degrees_per_second_t tolerance) {
  frc2::Trigger near = IsNear(velocity, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce(
             [this, velocity] {
               m_smc->StartClosedLoopController();
               m_smc->SetVelocity(velocity);
             },
             {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

frc2::CommandPtr FlyWheel::RunTo(std::function<units::degrees_per_second_t()> velocity,
                                 units::degrees_per_second_t tolerance) {
  units::degrees_per_second_t target = velocity();
  frc2::Trigger near = IsNear(target, tolerance).Debounce(units::second_t{0.1});
  return frc2::cmd::RunOnce(
             [this, target] {
               m_smc->StartClosedLoopController();
               m_smc->SetVelocity(target);
             },
             {m_subsystem})
      .AndThen(frc2::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

frc2::CommandPtr FlyWheel::RunTo(units::meters_per_second_t velocity,
                                 units::meters_per_second_t tolerance) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s RunTo: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return frc2::cmd::None();
  }
  units::meter_t radius = *diameter / 2.0;
  double convFactor = (180.0 / std::numbers::pi) / radius.value();
  return RunTo(units::degrees_per_second_t{velocity.value() * convFactor},
               units::degrees_per_second_t{tolerance.value() * convFactor});
}

frc2::CommandPtr FlyWheel::RunTo(std::function<units::meters_per_second_t()> velocity,
                                 units::meters_per_second_t tolerance) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s RunTo: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return frc2::cmd::None();
  }
  units::meter_t radius = *diameter / 2.0;
  double convFactor = (180.0 / std::numbers::pi) / radius.value();
  return RunTo(
      [velocity, convFactor] {
        return units::degrees_per_second_t{velocity().value() * convFactor};
      },
      units::degrees_per_second_t{tolerance.value() * convFactor});
}

frc2::Trigger FlyWheel::Gte(units::degrees_per_second_t velocity) {
  return frc2::Trigger{[this, velocity] { return GetVelocity() >= velocity; }};
}

frc2::Trigger FlyWheel::Lte(units::degrees_per_second_t velocity) {
  return frc2::Trigger{[this, velocity] { return GetVelocity() <= velocity; }};
}

frc2::Trigger FlyWheel::Between(units::degrees_per_second_t start,
                                units::degrees_per_second_t end) {
  return Gte(start).And(Lte(end));
}

frc2::Trigger FlyWheel::IsNear(units::degrees_per_second_t velocity,
                               units::degrees_per_second_t within) {
  return frc2::Trigger{[this, velocity, within] {
    return std::abs(GetVelocity().value() - velocity.value()) <= within.value();
  }};
}

void FlyWheel::SetVelocity(units::degrees_per_second_t velocity) {
  SetMechanismVelocitySetpoint(velocity);
}

void FlyWheel::SetSurfaceSpeed(units::meters_per_second_t speed) {
  SetMeasurementVelocitySetpoint(speed);
}

void FlyWheel::SetMeasurementVelocitySetpoint(units::meters_per_second_t velocity) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    return;
  }
  units::meter_t radius = *diameter / 2.0;
  SetMechanismVelocitySetpoint(units::degrees_per_second_t{(velocity.value() / radius.value()) *
                                                           (180.0 / std::numbers::pi)});
}

frc::Translation3d FlyWheel::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return frc::Translation3d{
        units::meter_t{m_mechanismLigament->GetLength()},
        frc::Rotation3d{0_rad, 0_rad, units::radian_t{m_mechanismLigament->GetAngle()}}};
  }
  return frc::Translation3d{};
}

const config::FlyWheelConfig& FlyWheel::GetConfig() const { return m_flyWheelConfig; }

units::degrees_per_second_t FlyWheel::GetVelocity() const { return m_smc->GetMechanismVelocity(); }

}  // namespace yams::mechanisms::velocity
