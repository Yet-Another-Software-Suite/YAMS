// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/ShooterSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/time.h>
#include <units/voltage.h>

#include <iostream>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ShooterSubsystem::ShooterSubsystem() {
  // kP=10 chosen empirically; at shooter speeds (thousands of deg/s) a small
  // error still produces substantial correction voltage without oscillating.
  // kI and kD are zero -- integral wind-up is not useful for a flywheel that
  // is always spinning, and derivative amplifies encoder noise.
  m_motorConfig.WithSubsystem(this)
      .WithFeedback(10, 0, 0)
      // 3 * 4 = 12:1 total reduction; SparkWrapper uses this to convert between
      // motor-shaft turns and mechanism (wheel) turns for velocity control.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      // COAST so the wheel spins down freely after a shot instead of hard-braking.
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithTelemetry("ShooterMotor", Cfg::TelemetryVerbosity::HIGH)
      .WithMotorInverted(false)
      // Feedforward zeroed for now; tune kV once actual free-spin RPM is measured.
      .WithSimpleFeedforward(0, 0, 0)
      .WithClosedLoopMode()
      // m_flywheelMotor2 mirrors m_flywheelMotor1 via hardware follower mode.
      // false = not inverted relative to the leader.
      .WithFollowers({{&m_flywheelMotor2, false}});

  // NEO(2) tells the simulation plant that two NEO motors are mechanically coupled.
  // At runtime only m_flywheelMotor1 receives commands; motor2 follows via CAN.
  m_motor.emplace(&m_flywheelMotor1, frc::DCMotor::NEO(2), &m_motorConfig);

  // 4-inch diameter wheel; 4.0 * 0.0254 converts inches to meters.
  // WithRollerDiameter enables SetSurfaceSpeedSetpoint (m/s at the wheel rim).
  m_shooterConfig.WithRollerDiameter(units::meter_t{4.0 * 0.0254}).WithTelemetryName("ShooterMech");

  m_shooter.emplace(&m_shooterConfig, &m_motor.value());
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

// Surface speed in m/s is converted internally by FlyWheel using the roller
// diameter set in m_shooterConfig. Useful when the upstream calculation works
// in linear ball speed rather than angular motor speed.
void ShooterSubsystem::SetSurfaceSpeedSetpoint(units::meters_per_second_t speed) {
  m_shooter->SetMeasurementVelocitySetpoint(speed);
}

bool ShooterSubsystem::ReadyToShoot(units::degrees_per_second_t tolerance) const {
  // IsNear returns an frc2::Trigger; .Get() evaluates it immediately (no
  // scheduler needed) -- fine for a synchronous boolean check in a command.
  return m_shooter->IsNear(GetVelocity(), tolerance).Get();
}

void ShooterSubsystem::Periodic() { m_shooter->UpdateTelemetry(); }

void ShooterSubsystem::SimulationPeriodic() { m_shooter->SimIterate(); }
