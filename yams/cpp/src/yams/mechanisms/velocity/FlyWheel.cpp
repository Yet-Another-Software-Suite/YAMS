// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/velocity/FlyWheel.h"

#include <frc/DriverStation.h>
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
    m_smc->SetEncoderPosition(ss->GetMechanismPosition());
    m_smc->SetEncoderVelocity(ss->GetMechanismVelocity());
    ss->FeedWatchdog();
  } else {
    m_smc->SimIterate();
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

// ---- FlyWheel-specific interface --------------------------------------------

frc2::CommandPtr FlyWheel::Spin(units::degrees_per_second_t velocity) {
  return frc2::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity); }, {m_subsystem})
      .WithName(m_name + " Spin");
}

frc2::CommandPtr FlyWheel::Spin(std::function<units::degrees_per_second_t()> velocity) {
  return frc2::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity()); },
                        {m_subsystem})
      .WithName(m_name + " Spin Supplier");
}

frc2::CommandPtr FlyWheel::SpinSurface(units::meters_per_second_t surfaceSpeed) {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr,
                 "[YAMS] %s SpinSurface: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return frc2::cmd::None();
  }

  // surface_speed = omega * radius  =>  omega = surface_speed / radius
  // radius = diameter / 2
  units::meter_t radius = *diameter / 2.0;

  // Convert m/s → deg/s:  omega_deg_s = (v / r) * (180 / pi)
  return frc2::cmd::Run(
             [this, surfaceSpeed, radius] {
               units::degrees_per_second_t angularVelocity{(surfaceSpeed.value() / radius.value()) *
                                                           (180.0 / std::numbers::pi)};
               SetMechanismVelocitySetpoint(angularVelocity);
             },
             {m_subsystem})
      .WithName(m_name + " SpinSurface");
}

units::degrees_per_second_t FlyWheel::GetVelocity() const { return m_smc->GetMechanismVelocity(); }

units::meters_per_second_t FlyWheel::GetSurfaceSpeed() const {
  auto diameter = m_flyWheelConfig.GetRollerDiameter();
  if (!diameter) {
    return units::meters_per_second_t{0.0};
  }
  units::meter_t radius = *diameter / 2.0;
  // v = omega * r  (convert deg/s → rad/s first)
  double omegaRadPerSec = GetVelocity().value() * (std::numbers::pi / 180.0);
  return units::meters_per_second_t{omegaRadPerSec * radius.value()};
}

bool FlyWheel::AtVelocity(units::degrees_per_second_t target,
                          units::degrees_per_second_t tolerance) const {
  return std::abs(GetVelocity().value() - target.value()) <= tolerance.value();
}

frc2::Trigger FlyWheel::IsAtVelocity(units::degrees_per_second_t target,
                                     units::degrees_per_second_t tolerance) {
  return frc2::Trigger{[this, target, tolerance] { return AtVelocity(target, tolerance); }};
}

}  // namespace yams::mechanisms::velocity
