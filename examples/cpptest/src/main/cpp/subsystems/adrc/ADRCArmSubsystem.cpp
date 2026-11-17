// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// An arm run by an ADRC, written by FRC Team 8331. See ADRCArmSubsystem.h.

#include "subsystems/adrc/ADRCArmSubsystem.h"

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

ADRCArmSubsystem::ADRCArmSubsystem() {
  ADRCConfig adrc;
  // A position loop on an arm; its order and b0 are chosen from the motor, gearing and inertia.
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::ARM)
      // How quickly the arm should reach its setpoint.
      .WithSettlingTime(0.3_s);

  m_motorConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      // The output limit and ramp of every closed loop, the ADRC's too.
      .WithClosedLoopMaxVoltage(12_V)
      // The ADRC takes the place of kP, kI, kD and the feedforward. The motor config keeps its own copy.
      .WithADRCController(adrc)
      .WithMotorGearing(60.0)
      // The ADRC's order and b0 are derived from the motor, the gearing and this inertia.
      .WithMOI(0.5_m, 3_kg)
      .WithMechanismLimits(-20_deg, 100_deg)
      .WithStartingPosition(-20_deg)
      .WithZeroPower(Cfg::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH);

  m_motor.emplace(&m_armMotor, wpi::math::DCMotor::NEO(1), &m_motorConfig);

  // Stowed on its lower hard stop, which is also its lower soft limit.
  m_armConfig.WithArmLength(0.5_m).WithMinAngle(-20_deg).WithMaxAngle(110_deg).WithTelemetryName("ADRCArm");
  m_arm.emplace(&m_armConfig, &m_motor.value());
}

wpi::cmd::CommandPtr ADRCArmSubsystem::SetAngle(wpi::units::degree_t angle) { return m_arm->Run(angle); }

wpi::units::degree_t ADRCArmSubsystem::GetAngle() const { return m_arm->GetAngle(); }

yams::math::ADRCController* ADRCArmSubsystem::GetADRC() { return m_motor->GetADRCController(); }

void ADRCArmSubsystem::Periodic() { m_arm->UpdateTelemetry(); }

void ADRCArmSubsystem::SimulationPeriodic() { m_arm->SimIterate(); }
