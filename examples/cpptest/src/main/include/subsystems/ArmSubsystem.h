// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/DigitalInput.h>
#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/TalonFX.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

// Single-jointed arm driven by a Kraken X60 (CAN 1) through a 3:1 x 4:1 (12:1 total)
// gearbox. Arm length is 0.135 m; the Arm mechanism uses this for gravity compensation
// via the kG term in WithFeedforward(ArmFeedforward) (currently zeroed out -- tune before use).
// A trapezoidal motion profile limits speed to 0.5 turns/s and accel to 0.25 turns/s^2.
//
// Angle limits: WithMechanismLimits [-30, +100] deg (motor-level soft limits);
//   ArmConfig adds a second layer at [-100, +200] deg for the simulation model.
// Starting position seeds to 0 deg on deploy; there is no absolute encoder wired
// into the feedback loop -- if the arm is not physically at 0 on deploy, re-home it
// before enabling closed-loop commands.
//
// The CANcoder (CAN 2) is declared but not yet passed to WithExternalEncoder in the
// .cpp; wire it in if you need absolute position seeding across power cycles.
//
// DIO 0 is a beam-break sensor. GetBeamBreak() is polled in Periodic() and can be
// used to detect a game piece in the arm intake path.
//
// Commands:
//   ArmCmd(dutycycle) -- open-loop percentage output
//   SetAngle(degree_t) -- closed-loop continuous move (trapezoidal profile active)
class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  bool GetBeamBreak();

  frc2::CommandPtr ArmCmd(double dutycycle);
  frc2::CommandPtr SetAngle(units::degree_t angle);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::CANcoder m_cancoder{
      2};  // absolute encoder, CAN 2 (not yet wired into feedback)
  ctre::phoenix6::hardware::TalonFX m_armMotor{1};  // Kraken X60, CAN 1

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::remote::TalonFXWrapper> m_motor;

  yams::mechanisms::config::ArmConfig m_armConfig;
  std::optional<yams::mechanisms::positional::Arm> m_arm;

  frc::DigitalInput m_dio{0};  // beam-break sensor on DIO 0
};
