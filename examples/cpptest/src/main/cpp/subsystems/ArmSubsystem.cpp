// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Single-jointed arm: Kraken X60 (CAN 1) + CANcoder (CAN 2) through a 12:1 gearbox.
// Travel is -30 to +100 deg (motor-controller soft limits); ArmConfig extends to
// [-100, 200] for the SingleJointedArmSim, which is wider intentionally (see below).
// A beam-break on DIO 0 detects game pieces but currently only runs as a polled read.

#include "subsystems/ArmSubsystem.h"

#include <frc/system/plant/DCMotor.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/time.h>

using namespace yams::motorcontrollers;
using namespace yams::gearing;
using namespace yams::mechanisms;
using Cfg = SmartMotorControllerConfig;

ArmSubsystem::ArmSubsystem() {
  m_motorConfig
      .WithSubsystem(this)
      // kP=4: stiff enough to hold position against gravity at 12:1 with a short arm
      // (~135 mm). If the arm vibrates at a setpoint, reduce kP or add kD; if it sags
      // under load, add kG in the feedforward below.
      .WithFeedback(4, 0, 0)
      // Trapezoid profile at 0.5 turn/s max velocity, 0.25 turn/s^2 max accel (mechanism
      // turns, not rotor turns). These are conservative -- the arm reaches any target in
      // the ~130 deg range in under 2 s. Tune up once feedforward is characterized.
      .WithTrapezoidProfile(units::turns_per_second_t{0.5}, units::turns_per_second_squared_t{0.25})
      // Motor-controller soft limits: -30 to +100 deg. These fire first and protect the
      // arm against code bugs. The ArmConfig limits below are wider and only bound the sim.
      .WithMechanismLimits(units::degree_t{-30}, units::degree_t{100})
      // 3:1 then 4:1 in series = 12:1 total. Wrong reduction here directly scales the
      // position setpoint error -- e.g. using 6:1 would make the arm overshoot by 2x.
      .WithMotorGearing(MechanismGearing{GearBox::FromReductionStages({3.0, 4.0})})
      // BRAKE holds the arm against gravity when idle; COAST would let it fall to the
      // lower soft limit and potentially stress the physical hard stop.
      .WithIdleMode(Cfg::MotorMode::BRAKE)
      .WithTelemetry("ArmMotor", Cfg::TelemetryVerbosity::HIGH)
      // Stator limit: 40 A prevents the motor from cooking the gearbox if the arm hits
      // a hard stop. Supply is left uncapped -- the stator limit is the binding one here.
      .WithStatorCurrentLimit(units::ampere_t{40})
      .WithMotorInverted(false)
      // 0.25 s ramp on both loops prevents voltage spikes when switching direction quickly
      // or when open-loop jogging near the soft limits. Remove or tighten once gains are set.
      .WithClosedLoopRampRate(units::second_t{0.25})
      .WithOpenLoopRampRate(units::second_t{0.25})
      // All feedforward terms at 0: placeholders for sysid. kS removes stiction, kV
      // improves velocity tracking through the profile, kG compensates gravity at the
      // current angle (cos(angle) * kG). Without kG the arm will droop at horizontal.
      .WithArmFeedforward(0, 0, 0, 0)
      // Arm is assumed horizontal (0 deg) at power-on. If the arm powers on in a known
      // tucked position, set this to match that angle so the first move is relative to
      // the correct starting point.
      .WithStartingPosition(units::degree_t{0})
      .WithClosedLoopMode();

  // KrakenX60(1): single-motor sim model. The arm has one motor driving the 12:1 box,
  // so motor count stays at 1 -- do not increase unless a second motor is added as a follower.
  m_motor.emplace(&m_armMotor, frc::DCMotor::KrakenX60(1), &m_motorConfig);

  // ArmConfig feeds the SingleJointedArmSim; arm length drives MOI estimation (1/3 * m * L^2).
  // 0.135 m is the distance from the pivot to the center of mass, not the total arm length.
  // Getting this wrong in sim makes the simulated inertia unrealistic but has no hardware effect.
  m_armConfig
      .WithArmLength(units::meter_t{0.135})
      // Sim limits are wider than the motor-controller soft limits (-30 to +100) so the
      // sim does not clip before the soft-limit logic fires. Do not use these as the
      // canonical travel bounds -- those are in WithMechanismLimits above.
      .WithMinAngle(units::degree_t{-100})
      .WithMaxAngle(units::degree_t{200})
      .WithTelemetryName("ArmExample");

  m_arm.emplace(&m_armConfig, &m_motor.value());
}

bool ArmSubsystem::GetBeamBreak() { return m_dio.Get(); }

void ArmSubsystem::Periodic() {
  // GetBeamBreak() result is currently discarded -- wire it to a member flag or trigger
  // when game-piece detection needs to drive commands.
  GetBeamBreak();
  m_arm->UpdateTelemetry();
}

void ArmSubsystem::SimulationPeriodic() { m_arm->SimIterate(); }

frc2::CommandPtr ArmSubsystem::ArmCmd(double dutycycle) { return m_arm->Set(dutycycle); }

frc2::CommandPtr ArmSubsystem::SetAngle(units::degree_t angle) { return m_arm->Run(angle); }
