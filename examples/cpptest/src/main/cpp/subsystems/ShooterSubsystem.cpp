// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/ShooterSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ShooterSubsystem::ShooterSubsystem() {
  m_motorConfig.WithSubsystem(this)
      .WithFeedback(1, 0, 0)
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithMotorInverted(false)
      .WithSimpleFeedforward(0, 0, 0)
      .WithClosedLoopMode();

  m_motor.emplace(m_flywheelMotor1, frc::DCMotor::NEO(2), m_motorConfig);

  // 4-inch diameter flywheel wheel
  m_shooterConfig.WithMotorController(&m_motor.value())
      .WithSubsystem(this)
      .WithRollerDiameter(units::meter_t{4.0 * 0.0254})
      .WithTelemetryName("ShooterMech");

  m_shooter.emplace(m_shooterConfig);
}

units::degrees_per_second_t ShooterSubsystem::GetVelocity() const {
  return m_shooter->GetVelocity();
}

frc2::CommandPtr ShooterSubsystem::SetVelocity(units::degrees_per_second_t speed) {
  return m_shooter->Run(speed);
}

frc2::CommandPtr ShooterSubsystem::SetVelocity(std::function<units::degrees_per_second_t()> speed) {
  return m_shooter->Run(speed);
}

frc2::CommandPtr ShooterSubsystem::Set(double dutyCycle) { return m_shooter->Set(dutyCycle); }

frc2::CommandPtr ShooterSubsystem::Set(std::function<double()> dutyCycle) {
  return m_shooter->Set(dutyCycle);
}

void ShooterSubsystem::SetVelocitySetpoint(units::degrees_per_second_t speed) {
  m_shooter->SetMechanismVelocitySetpoint(speed);
}

void ShooterSubsystem::SetDutyCycleSetpoint(double dutyCycle) {
  m_shooter->SetDutyCycleSetpoint(dutyCycle);
}

void ShooterSubsystem::SetSurfaceSpeedSetpoint(units::meters_per_second_t speed) {
  m_shooter->SetMeasurementVelocitySetpoint(speed);
}

bool ShooterSubsystem::ReadyToShoot(units::degrees_per_second_t tolerance) const {
  return m_shooter->IsNear(GetVelocity(), tolerance).Get();
}

void ShooterSubsystem::Periodic() { m_shooter->UpdateTelemetry(); }

void ShooterSubsystem::SimulationPeriodic() { m_shooter->SimIterate(); }
