// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/ElevatorSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <numbers>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ElevatorSubsystem::ElevatorSubsystem() {
  // Chain pitch 0.25 in, 22 teeth → circumference = 0.25 * 22 = 5.5 in
  constexpr double kChainPitchIn = 0.25;
  constexpr int kToothCount = 22;
  constexpr double kCircumferenceIn = kChainPitchIn * kToothCount;
  const units::meter_t circumference{kCircumferenceIn * 0.0254};

  m_motorConfig.WithSubsystem(this)
      .WithMechanismCircumference(circumference)
      .WithFeedback(30, 0, 0)
      .WithExponentialProfile(0.0, 0.0, units::volt_t{12})
      .WithMeasurementLimits(units::meter_t{0}, units::meter_t{2})
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithMotorInverted(false)
      .WithElevatorFeedforward(0, 0, 0)
      .WithClosedLoopMode();

  m_motor.emplace(m_elevatorMotor, frc::DCMotor::NEO(1), m_motorConfig);

  m_elevatorConfig.WithMotorController(&m_motor.value())
      .WithSubsystem(this)
      .WithStartingHeight(units::meter_t{0.5})
      .WithMinimumHeight(units::meter_t{0})
      .WithMaximumHeight(units::meter_t{3})
      .WithTelemetryName("Elevator");

  m_elevator.emplace(m_elevatorConfig);
}

void ElevatorSubsystem::Periodic() { m_elevator->UpdateTelemetry(); }

void ElevatorSubsystem::SimulationPeriodic() { m_elevator->SimIterate(); }

frc2::CommandPtr ElevatorSubsystem::ElevCmd(double dutycycle) { return m_elevator->Set(dutycycle); }

frc2::CommandPtr ElevatorSubsystem::SetHeight(units::meter_t height) {
  return m_elevator->GoToHeight(height);
}

frc2::CommandPtr ElevatorSubsystem::SysId() {
  return m_elevator->SysId(units::volt_t{12}, frc2::sysid::ramp_rate_t{12.0}, units::second_t{30});
}
