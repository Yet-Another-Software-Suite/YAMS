// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/ArmSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>
#include <units/voltage.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ArmSubsystem::ArmSubsystem() {
  m_motorConfig.WithSubsystem(this)
      .WithFeedback(4, 0, 0)
      .WithTrapezoidProfile(units::turns_per_second_t{0.5}, units::turns_per_second_squared_t{0.25})
      .WithMechanismLimits(units::degree_t{-30}, units::degree_t{100})
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithMotorInverted(false)
      .WithClosedLoopRampRate(units::second_t{0.25})
      .WithOpenLoopRampRate(units::second_t{0.25})
      .WithArmFeedforward(0, 0, 0, 0)
      .WithClosedLoopMode();

  m_motor.emplace(m_armMotor, frc::DCMotor::KrakenX60(1), m_motorConfig);
  m_motor->WithCANcoder(m_cancoder);

  m_armConfig.WithMotorController(&m_motor.value())
      .WithSubsystem(this)
      .WithArmLength(units::meter_t{0.135})
      .WithMinAngle(units::degree_t{-100})
      .WithMaxAngle(units::degree_t{200})
      .WithTelemetryName("ArmExample")
      .WithStartingAngle(units::degree_t{0});

  m_arm.emplace(m_armConfig);
}

bool ArmSubsystem::GetBeamBreak() { return m_dio.Get(); }

void ArmSubsystem::Periodic() {
  GetBeamBreak();
  m_arm->UpdateTelemetry();
}

void ArmSubsystem::SimulationPeriodic() { m_arm->SimIterate(); }

frc2::CommandPtr ArmSubsystem::ArmCmd(double dutycycle) { return m_arm->Set(dutycycle); }

frc2::CommandPtr ArmSubsystem::SysId() {
  return m_arm->SysId(units::volt_t{3}, frc2::sysid::ramp_rate_t{3.0}, units::second_t{30});
}

frc2::CommandPtr ArmSubsystem::SetAngle(units::degree_t angle) { return m_arm->Run(angle); }
