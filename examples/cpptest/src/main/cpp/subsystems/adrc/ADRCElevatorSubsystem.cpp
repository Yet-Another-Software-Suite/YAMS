// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// An elevator run by an ADRC, written by FRC Team 8331. See ADRCElevatorSubsystem.h.

#include "subsystems/adrc/ADRCElevatorSubsystem.h"

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/units/current.hpp>
#include <wpi/units/length.hpp>
#include <wpi/units/mass.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/voltage.hpp>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/math/ADRCConfig.hpp"

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using yams::math::ADRCConfig;
using Cfg = SmartMotorControllerConfig;

ADRCElevatorSubsystem::ADRCElevatorSubsystem() {
  // 3:1 then 4:1, 12 motor rotations per sprocket rotation.
  const MechanismGearing gearing{GearBox::FromReductionStages({3.0, 4.0})};
  const auto motor = wpi::math::DCMotor::NEO(1);
  // A 22 tooth sprocket on 1/4 in pitch chain moves the carriage 5.5 in per rotation.
  const wpi::units::meter_t circumference = 0.25_in * 22;

  ADRCConfig adrc;
  // First order on position, with b0 its speed per volt from the motor, gearing and sprocket.
  adrc.WithSystemOrder(ADRCConfig::SystemOrder::ELEVATOR).WithSettlingTime(0.3_s);

  m_motorConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      // The output limit and ramp of every closed loop, the ADRC's too.
      .WithClosedLoopMaxVoltage(12_V)
      // The ADRC takes the place of kP, kI, kD and the feedforward. The motor config keeps its own copy.
      .WithADRCController(adrc)
      .WithMotorGearing(gearing)
      .WithMechanismCircumference(circumference)
      // The closed loop, and the ADRC, work in meters, as SystemOrder::ELEVATOR needs.
      .WithLinearClosedLoopController(true)
      .WithMeasurementLimits(0_m, 1.5_m)
      .WithStartingPosition(0_m)
      .WithZeroPower(Cfg::MotorMode::BRAKE)
      .WithStatorCurrentLimit(40_A)
      .WithTelemetry("ElevatorMotor", Cfg::TelemetryVerbosity::HIGH);

  m_motor.emplace(&m_elevatorMotor, motor, &m_motorConfig);

  // Starts on its lower hard stop.
  m_elevatorConfig.WithCarriageMass(16_lb)
      .WithMinimumHeight(0_m)
      .WithMaximumHeight(1.6_m)
      .WithTelemetryName("ADRCElevator");
  m_elevator.emplace(&m_elevatorConfig, &m_motor.value());
}

wpi::cmd::CommandPtr ADRCElevatorSubsystem::SetHeight(wpi::units::meter_t height) { return m_elevator->Run(height); }

wpi::units::meter_t ADRCElevatorSubsystem::GetHeight() const { return m_elevator->GetHeight(); }

yams::math::ADRCController* ADRCElevatorSubsystem::GetADRC() { return m_motor->GetADRCController(); }

void ADRCElevatorSubsystem::Periodic() { m_elevator->UpdateTelemetry(); }

void ADRCElevatorSubsystem::SimulationPeriodic() { m_elevator->SimIterate(); }
