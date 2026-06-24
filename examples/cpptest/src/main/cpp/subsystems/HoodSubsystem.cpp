// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "subsystems/HoodSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;
using TalonFXSWrapper = yams::motorcontrollers::remote::TalonFXSWrapper;

HoodSubsystem::HoodSubsystem() {
  // The TalonFXS (CAN 9) is a CTRE controller wired to a REV NEO.  MotorArrangement::NEO
  // below tells the TalonFXS which commutation table and current limits to use for the NEO.
  // All SmartMotorControllerConfig options here go into that wrapper via ApplyConfig().

  // kP=4 is the only active PID term; kI and kD are both 0.  The trapezoid profile caps
  // angular speed at 0.5 turns/s and acceleration at 0.25 turns/s^2, so the hood accelerates
  // to full speed in ~2 s.  These are conservative starting values -- tune kP and the profile
  // together: too-high kP with a slow profile produces oscillation on arrival.
  //
  // WithMechanismLimits sets TalonFXS hardware soft limits (-30 deg to +100 deg).  The
  // controller enforces these in firmware, so they take effect even if robot code hangs.
  //
  // GearBox::FromReductionStages({3.0, 4.0}): two sequential reductions, 3:1 then 4:1, giving
  // a 12:1 total.  One motor turn moves the hood 1/12 of a turn (30 deg).
  //
  // WithArmFeedforward(0, 0, 0, 0): kS, kV, kA, kG all zeroed out.  kG should be non-zero if
  // the hood is not balanced (gravity torque changes with angle).  Set it via system ID or
  // manual tuning: kG = voltage needed to hold the hood at 90 deg from horizontal.
  //
  // Ramp rates of 0.25 s prevent step-voltage surges on the NEO in both open- and closed-loop.
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
      .WithStartingPosition(units::degree_t{0})
      .WithClosedLoopMode();

  // emplace() deferred until here so m_hoodMotor is fully constructed before we take its address.
  // MotorArrangement::NEO selects the NEO brushless commutation profile inside the TalonFXS.
  m_motor.emplace(&m_hoodMotor, frc::DCMotor::NEO(1), TalonFXSWrapper::MotorArrangement::NEO,
                  &m_motorConfig);

  // PivotConfig wires the motor controller to the Pivot mechanism.  WithMin/MaxAngle here are
  // simulation hard-stop bounds used by DCMotorSim -- wider than the firmware soft limits above
  // so the sim doesn't clamp before firmware would.  WithTelemetryName sets the NT4 table key.
  m_pivotConfig.WithMinAngle(units::degree_t{-100})
      .WithMaxAngle(units::degree_t{200})
      .WithTelemetryName("HoodExample");

  m_hood.emplace(&m_pivotConfig, &m_motor.value());
}

// Non-command path for periodic callers that manage their own setpoint scheduling.
// Writes straight to the motor's closed-loop target each loop -- no command overhead.
void HoodSubsystem::SetAngleSetpoint(units::degree_t angle) {
  m_hood->SetMechanismPositionSetpoint(angle);
}

// Publishes mechanism angle, velocity, and setpoint to NT4 each loop.
void HoodSubsystem::Periodic() { m_hood->UpdateTelemetry(); }

// Advances the DCMotorSim by one 20 ms loop tick.  Only runs in simulation.
void HoodSubsystem::SimulationPeriodic() { m_hood->SimIterate(); }

// Open-loop override: bypasses the trapezoid profile and sends duty-cycle directly.
// Useful for manual tuning or operator override during testing.
frc2::CommandPtr HoodSubsystem::HoodCmd(double dutycycle) { return m_hood->Set(dutycycle); }

// Closed-loop position command.  Pivot::Run() runs indefinitely (until interrupted).
// Use Pivot::RunTo() instead if the command should end once within tolerance.
frc2::CommandPtr HoodSubsystem::SetAngle(units::degree_t angle) { return m_hood->Run(angle); }
