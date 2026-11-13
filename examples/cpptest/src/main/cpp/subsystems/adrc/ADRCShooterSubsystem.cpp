// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// A shooter run by an ADRC, written by FRC Team 8331. See ADRCShooterSubsystem.h.

#include "subsystems/adrc/ADRCShooterSubsystem.h"

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/math/ADRCConfig.hpp"

using namespace yams::motorcontrollers;
using namespace yams::mechanisms;
using yams::math::ADRCConfig;
using Cfg = SmartMotorControllerConfig;

ADRCShooterSubsystem::ADRCShooterSubsystem() {
  ADRCConfig adrc;
  // A velocity loop on a flywheel is first order.
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::FLYWHEEL)
      // Short, so the observer keeps up with the back-EMF it estimates.
      .WithSettlingTime(0.15_s);

  m_motorConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      // The output limit and ramp of every closed loop, the ADRC's too.
      .WithClosedLoopMaxVoltage(12_V)
      // The ADRC takes the place of kP, kI, kD and the feedforward. The motor config keeps its own copy.
      .WithADRCController(adrc)
      .WithMotorGearing(1.0)
      // b0, how much one volt accelerates the wheel, is derived from the motor, the gearing and this inertia.
      .WithWheelMOI(4_in, 1_lb)
      .WithZeroPower(Cfg::MotorMode::COAST)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH);

  m_motor.emplace(&m_shooterMotor, wpi::math::DCMotor::NEO(1), &m_motorConfig);

  m_shooterConfig.WithRollerDiameter(4_in).WithTelemetryName("ADRCShooter");
  m_shooter.emplace(&m_shooterConfig, &m_motor.value());

  SetDefaultCommand(m_shooter->Set(0.0));
}

wpi::cmd::CommandPtr ADRCShooterSubsystem::SpinUp(wpi::units::degrees_per_second_t speed) {
  return m_shooter->Run(speed);
}

wpi::units::degrees_per_second_t ADRCShooterSubsystem::GetSpeed() const { return m_shooter->GetVelocity(); }

yams::math::ADRCController* ADRCShooterSubsystem::GetADRC() { return m_motor->GetADRCController(); }

void ADRCShooterSubsystem::Periodic() { m_shooter->UpdateTelemetry(); }

void ADRCShooterSubsystem::SimulationPeriodic() { m_shooter->SimIterate(); }
