// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

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

// Two independent Kraken X60 motors: lower (CAN 4) and upper (CAN 6).
// Each wheel can be set to a different speed, which puts asymmetric backspin/topspin
// on the game piece for spin-control shooting.
DoubleFlyWheelSubsystem::DoubleFlyWheelSubsystem() {
  // kP=1 is low because Kraken velocity control runs at 1 kHz internally via
  // MotionMagic/VelocityTorqueCurrentFOC; even a small gain produces tight tracking.
  // kI and kD left at zero -- same reasoning as single-wheel shooter.
  m_lowerConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      // COAST: lets the wheel spin down between shots rather than fighting back-EMF.
      .WithIdleMode(Cfg::MotorMode::COAST)
      // 3 * 4 = 12:1 reduction between Kraken shaft and wheel shaft.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      // MOI measured for the physical flywheel disk; simulation uses this for
      // accurate velocity step-response without tuning purely in software.
      .WithMOI(units::kilogram_square_meter_t{0.00029264})
      .WithFeedback(1, 0, 0)
      // Feedforward at zero for now; kV/kS should be measured on the real robot
      // using SysId before closed-loop tuning.
      .WithFeedforward(frc::SimpleMotorFeedforward<units::turns>{
          units::volt_t{0}, units::unit_t<frc::SimpleMotorFeedforward<units::turns>::kv_unit>{0},
          units::unit_t<frc::SimpleMotorFeedforward<units::turns>::ka_unit>{0}})
      .WithMotorInverted(false)
      .WithTelemetry("LowerFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  m_upperConfig.WithSubsystem(this)
      .WithClosedLoopMode()
      .WithIdleMode(Cfg::MotorMode::COAST)
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      .WithMOI(units::kilogram_square_meter_t{0.00029264})
      .WithFeedback(1, 0, 0)
      .WithFeedforward(frc::SimpleMotorFeedforward<units::turns>{
          units::volt_t{0}, units::unit_t<frc::SimpleMotorFeedforward<units::turns>::kv_unit>{0},
          units::unit_t<frc::SimpleMotorFeedforward<units::turns>::ka_unit>{0}})
      .WithMotorInverted(false)
      .WithTelemetry("UpperFlyWheel", Cfg::TelemetryVerbosity::HIGH);

  // TalonFXWrapper takes TalonFX* (pointer), not a reference -- pass address of
  // the member objects declared in the header.
  m_lowerFlyWheel.emplace(&m_talonLower, frc::DCMotor::KrakenX60(1), &m_lowerConfig);
  m_upperFlyWheel.emplace(&m_talonUpper, frc::DCMotor::KrakenX60(1), &m_upperConfig);
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetDutyCycle(double lower, double upper) {
  return Run([this, lower, upper] {
           m_lowerFlyWheel->SetDutyCycle(lower);
           m_upperFlyWheel->SetDutyCycle(upper);
         })
      .WithName("Set Duty Cycle (Double FlyWheel)");
}

frc2::CommandPtr DoubleFlyWheelSubsystem::SetVoltage(units::volt_t lower, units::volt_t upper) {
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
