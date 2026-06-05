// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/doubleflywheel/DoubleFlyWheelSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/Commands.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using Cfg = SmartMotorControllerConfig;

DoubleFlyWheelSubsystem::DoubleFlyWheelSubsystem() {
  m_lowerConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithMOI(units::kilogram_square_meter_t{0.00029264})
      .WithFeedback(1, 0, 0)
      .WithSimpleFeedforward(0, 0, 0)
      .WithMotorInverted(false)
      .WithTelemetry("LowerFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  m_upperConfig
      .WithSubsystem(this)
      .WithClosedLoopMode()
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithMOI(units::kilogram_square_meter_t{0.00029264})
      .WithFeedback(1, 0, 0)
      .WithSimpleFeedforward(0, 0, 0)
      .WithMotorInverted(false)
      .WithTelemetry("UpperFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  m_lowerFlyWheel.emplace(m_talonLower, frc::DCMotor::KrakenX60(1), m_lowerConfig);
  m_upperFlyWheel.emplace(m_talonUpper, frc::DCMotor::KrakenX60(1), m_upperConfig);
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetDutyCycle(double lower, double upper) {
  return Run([this, lower, upper] {
           m_lowerFlyWheel->SetDutyCycle(lower);
           m_upperFlyWheel->SetDutyCycle(upper);
         })
      .WithName("Set Duty Cycle (Double FlyWheel)");
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetVoltage(units::volt_t lower,
                                                      units::volt_t upper) {
  return Run([this, lower, upper] {
           m_lowerFlyWheel->SetVoltage(lower);
           m_upperFlyWheel->SetVoltage(upper);
         })
      .WithName("Set Voltage (Double FlyWheel)");
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetVelocity(units::degrees_per_second_t lower,
                                                       units::degrees_per_second_t upper) {
  return Run([this, lower, upper] {
    m_lowerFlyWheel->SetVelocity(lower);
    m_upperFlyWheel->SetVelocity(upper);
  });
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetVelocity(
    std::function<units::degrees_per_second_t()> lower,
    std::function<units::degrees_per_second_t()> upper) {
  return Run([this, lower, upper] {
    m_lowerFlyWheel->SetVelocity(lower());
    m_upperFlyWheel->SetVelocity(upper());
  });
}

void DoubleFlyWheelSubsystem::Periodic() {}

void DoubleFlyWheelSubsystem::SimulationPeriodic() {
  m_lowerFlyWheel->SimIterate();
  m_upperFlyWheel->SimIterate();
}
