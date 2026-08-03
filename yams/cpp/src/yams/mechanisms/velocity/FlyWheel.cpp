// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/velocity/FlyWheel.hpp"

#include <wpi/driverstation/DriverStation.hpp>
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
#include <wpi/util/Color8Bit.hpp>
#include <wpi/commands2/Commands.hpp>
#include <wpi/units/angle.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/math.hpp>
#include <wpi/units/time.hpp>

#include <cmath>
#include <cstdio>
#include <memory>
#include <numbers>
#include <string>

#include "yams/gearing/MechanismGearing.hpp"
#include "yams/motorcontrollers/simulation/DCMotorSimSupplier.hpp"

namespace yams::mechanisms::velocity {

// ---- Constructor ------------------------------------------------------------

FlyWheel::FlyWheel(config::FlyWheelConfig* config, motorcontrollers::SmartMotorController* smc)
    : SmartVelocityMechanism() {
  m_flyWheelConfig = config;
  m_smc = smc;
  m_subsystem = m_smc->GetConfig().GetSubsystem();

  if (!m_flyWheelConfig->GetTelemetryName().empty()) {
    m_name = m_flyWheelConfig->GetTelemetryName();
  }

  if (wpi::RobotBase::IsSimulation()) {
    // Create the DCMotorSim using LinearSystemId, then wire up the DCMotorSimSupplier.
    wpi::math::DCMotor dcMotor = m_smc->GetDCMotor();
    auto& gearingOpt = m_smc->GetConfig().GetMotorGearing();
    gearing::MechanismGearing gearing = gearingOpt.value_or(gearing::MechanismGearing::kOne);

    auto plant = wpi::math::Models::SingleJointedArmFromPhysicalConstants(dcMotor, m_smc->GetConfig().GetMOI(),
                                                    gearing.GetMechanismToRotorRatio());
    m_dcMotorSim.emplace(plant, dcMotor);

    wpi::units::second_t period = m_smc->GetConfig().GetClosedLoopControlPeriod().value_or(20_ms);
    m_smc->SetSimSupplier(std::make_shared<yams::motorcontrollers::simulation::DCMotorSimSupplier>(
        *m_dcMotorSim, [this]() { return m_smc->GetDutyCycle(); }, gearing, period));

    // Size the window from the configured roller diameter; default to 36 in (0.9144 m) like Java.
    wpi::units::meter_t shooterLength =
        m_flyWheelConfig->GetRollerDiameter().value_or(wpi::units::meter_t{0.9144});

    double len = shooterLength.value();
    m_mechanismWindow.emplace(len * 2.0, len * 2.0);
    m_mechanismRoot = m_mechanismWindow->GetRoot(m_name + "Root", len, len);
    m_mechanismLigament = m_mechanismRoot->Append<wpi::MechanismLigament2d>(
        m_name, len, 0_deg, 6, m_flyWheelConfig->GetSimColor());

    wpi::SmartDashboard::PutData(m_name + "/mechanism", &(*m_mechanismWindow));
  }
}

// ---- SmartMechanism overrides -----------------------------------------------

void FlyWheel::SimIterate() {
  if (auto* ss = m_smc->GetSimSupplier()) {
    ss->UpdateSim();
    m_smc->SimIterate();
    ss->StarveWatchdog();

    wpi::sim::RoboRioSim::SetVInVoltage(
        wpi::sim::BatterySim::Calculate({ss->GetCurrentDrawAmps()}));
    VisualizationUpdate();
  }
}

void FlyWheel::UpdateTelemetry() { m_smc->UpdateTelemetry(); }

void FlyWheel::VisualizationUpdate() {
  if (!m_mechanismLigament) {
    return;
  }

  m_mechanismLigament->SetAngle(wpi::units::degree_t{m_smc->GetMechanismPosition()});
}

std::string FlyWheel::GetName() const { return m_name; }

// ---- SmartVelocityMechanism overrides ---------------------------------------

wpi::cmd::Trigger FlyWheel::Max() {
  // FlyWheel has no inherent max velocity limit by default; always false.
  return wpi::cmd::Trigger{[] { return false; }};
}

wpi::cmd::Trigger FlyWheel::Min() {
  // FlyWheel has no inherent min velocity limit by default; always false.
  return wpi::cmd::Trigger{[] { return false; }};
}

// ---- Run / RunTo / comparison / setpoint interface --------------------------

wpi::cmd::CommandPtr FlyWheel::Run(wpi::units::degrees_per_second_t velocity) {
  return wpi::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity); }, {m_subsystem})
      .WithName(m_name + " Run");
}

wpi::cmd::CommandPtr FlyWheel::Run(std::function<wpi::units::degrees_per_second_t()> velocity) {
  return wpi::cmd::Run([this, velocity] { SetMechanismVelocitySetpoint(velocity()); },
                        {m_subsystem})
      .WithName(m_name + " Run Supplier");
}

wpi::cmd::CommandPtr FlyWheel::Run(wpi::units::meters_per_second_t surfaceSpeed) {
  auto diameter = m_flyWheelConfig->GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s Run: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return wpi::cmd::None();
  }
  wpi::units::meter_t radius = *diameter / 2.0;
  return wpi::cmd::Run(
             [this, surfaceSpeed, radius] {
               SetMechanismVelocitySetpoint(wpi::units::degrees_per_second_t{
                   (surfaceSpeed.value() / radius.value()) * (180.0 / std::numbers::pi)});
             },
             {m_subsystem})
      .WithName(m_name + " Run Surface");
}

wpi::cmd::CommandPtr FlyWheel::Run(std::function<wpi::units::meters_per_second_t()> surfaceSpeed) {
  auto diameter = m_flyWheelConfig->GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s Run: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return wpi::cmd::None();
  }
  wpi::units::meter_t radius = *diameter / 2.0;
  return wpi::cmd::Run(
             [this, surfaceSpeed, radius] {
               SetMechanismVelocitySetpoint(wpi::units::degrees_per_second_t{
                   (surfaceSpeed().value() / radius.value()) * (180.0 / std::numbers::pi)});
             },
             {m_subsystem})
      .WithName(m_name + " Run Surface Supplier");
}

wpi::cmd::CommandPtr FlyWheel::RunTo(wpi::units::degrees_per_second_t velocity,
                                 wpi::units::degrees_per_second_t tolerance) {
  wpi::cmd::Trigger near = IsNear(velocity, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce(
             [this, velocity] {
               m_smc->StartClosedLoopController();
               m_smc->SetVelocity(velocity);
             },
             {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo");
}

wpi::cmd::CommandPtr FlyWheel::RunTo(std::function<wpi::units::degrees_per_second_t()> velocity,
                                 wpi::units::degrees_per_second_t tolerance) {
  wpi::units::degrees_per_second_t target = velocity();
  wpi::cmd::Trigger near = IsNear(target, tolerance).Debounce(wpi::units::second_t{0.1});
  return wpi::cmd::RunOnce(
             [this, target] {
               m_smc->StartClosedLoopController();
               m_smc->SetVelocity(target);
             },
             {m_subsystem})
      .AndThen(wpi::cmd::WaitUntil([near] { return near.Get(); }))
      .WithName(m_name + " RunTo Supplier");
}

wpi::cmd::CommandPtr FlyWheel::RunTo(wpi::units::meters_per_second_t velocity,
                                 wpi::units::meters_per_second_t tolerance) {
  auto diameter = m_flyWheelConfig->GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s RunTo: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return wpi::cmd::None();
  }
  wpi::units::meter_t radius = *diameter / 2.0;
  double convFactor = (180.0 / std::numbers::pi) / radius.value();
  return RunTo(wpi::units::degrees_per_second_t{velocity.value() * convFactor},
               wpi::units::degrees_per_second_t{tolerance.value() * convFactor});
}

wpi::cmd::CommandPtr FlyWheel::RunTo(std::function<wpi::units::meters_per_second_t()> velocity,
                                 wpi::units::meters_per_second_t tolerance) {
  auto diameter = m_flyWheelConfig->GetRollerDiameter();
  if (!diameter) {
    std::fprintf(stderr, "[YAMS] %s RunTo: no roller diameter configured, command is a no-op.\n",
                 m_name.c_str());
    return wpi::cmd::None();
  }
  wpi::units::meter_t radius = *diameter / 2.0;
  double convFactor = (180.0 / std::numbers::pi) / radius.value();
  return RunTo(
      [velocity, convFactor] {
        return wpi::units::degrees_per_second_t{velocity().value() * convFactor};
      },
      wpi::units::degrees_per_second_t{tolerance.value() * convFactor});
}

wpi::cmd::Trigger FlyWheel::Gte(wpi::units::degrees_per_second_t velocity) {
  return wpi::cmd::Trigger{[this, velocity] { return GetVelocity() >= velocity; }};
}

wpi::cmd::Trigger FlyWheel::Lte(wpi::units::degrees_per_second_t velocity) {
  return wpi::cmd::Trigger{[this, velocity] { return GetVelocity() <= velocity; }};
}

wpi::cmd::Trigger FlyWheel::Between(wpi::units::degrees_per_second_t start,
                                wpi::units::degrees_per_second_t end) {
  return Gte(start) && (Lte(end));
}

wpi::cmd::Trigger FlyWheel::IsNear(wpi::units::degrees_per_second_t velocity,
                               wpi::units::degrees_per_second_t within) const {
  return wpi::cmd::Trigger{[this, velocity, within] {
    return std::abs(GetVelocity().value() - velocity.value()) <= within.value();
  }};
}

void FlyWheel::SetVelocity(wpi::units::degrees_per_second_t velocity) {
  SetMechanismVelocitySetpoint(velocity);
}

void FlyWheel::SetSurfaceSpeed(wpi::units::meters_per_second_t speed) {
  SetMeasurementVelocitySetpoint(speed);
}

void FlyWheel::SetMeasurementVelocitySetpoint(wpi::units::meters_per_second_t velocity) {
  auto diameter = m_flyWheelConfig->GetRollerDiameter();
  if (!diameter) {
    return;
  }
  wpi::units::meter_t radius = *diameter / 2.0;
  SetMechanismVelocitySetpoint(wpi::units::degrees_per_second_t{(velocity.value() / radius.value()) *
                                                           (180.0 / std::numbers::pi)});
}

wpi::math::Translation3d FlyWheel::GetRelativeMechanismPosition() const {
  if (m_mechanismLigament) {
    return wpi::math::Translation3d{
        wpi::units::meter_t{m_mechanismLigament->GetLength()},
        wpi::math::Rotation3d{0_rad, 0_rad, wpi::units::radian_t{m_mechanismLigament->GetAngle()}}};
  }
  return wpi::math::Translation3d{};
}

const config::FlyWheelConfig& FlyWheel::GetConfig() const { return *m_flyWheelConfig; }

wpi::units::degrees_per_second_t FlyWheel::GetVelocity() const { return m_smc->GetMechanismVelocity(); }

}  // namespace yams::mechanisms::velocity
