// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/HoodSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;
using TalonFXSWrapper = yams::motorcontrollers::remote::TalonFXSWrapper;

HoodSubsystem::HoodSubsystem() {
  m_motorConfig.WithSubsystem(this)
      .WithFeedback(4, 0, 0)
      .WithTrapezoidProfile(units::turns_per_second_t{0.5}, units::turns_per_second_squared_t{0.25})
      .WithMechanismLimits(units::degree_t{-30}, units::degree_t{100})
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithTelemetry("HoodMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithMotorInverted(false)
      .WithClosedLoopRampRate(units::second_t{0.25})
      .WithOpenLoopRampRate(units::second_t{0.25})
      .WithArmFeedforward(0, 0, 0, 0)
      .WithClosedLoopMode();

  m_motor.emplace(m_hoodMotor, frc::DCMotor::NEO(1), TalonFXSWrapper::MotorArrangement::NEO,
                  m_motorConfig);

  m_pivotConfig.WithMotorController(&m_motor.value())
      .WithSubsystem(this)
      .WithMinAngle(units::degree_t{-100})
      .WithMaxAngle(units::degree_t{200})
      .WithTelemetryName("HoodExample")
      .WithStartingAngle(units::degree_t{0});

  m_hood.emplace(m_pivotConfig);
}

void HoodSubsystem::SetAngleSetpoint(units::degree_t angle) {
  m_hood->SetMechanismPositionSetpoint(angle);
}

void HoodSubsystem::Periodic() { m_hood->UpdateTelemetry(); }

void HoodSubsystem::SimulationPeriodic() { m_hood->SimIterate(); }

frc2::CommandPtr HoodSubsystem::HoodCmd(double dutycycle) { return m_hood->Set(dutycycle); }

frc2::CommandPtr HoodSubsystem::SysId() {
  return m_hood->SysId(units::volt_t{3}, frc2::sysid::ramp_rate_t{3.0}, units::second_t{30});
}

frc2::CommandPtr HoodSubsystem::SetAngle(units::degree_t angle) { return m_hood->Run(angle); }
