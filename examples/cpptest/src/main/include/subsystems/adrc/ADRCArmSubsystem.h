// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <rev/SparkMax.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/units/angle.hpp>

#include <optional>

#include "yams/math/ADRCController.hpp"
#include "yams/mechanisms/config/ArmConfig.hpp"
#include "yams/mechanisms/positional/Arm.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// An arm run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO (CAN 30) on a
// 60:1 reduction, moving a 3 kg, 0.5 m arm. The ADRC was written by FRC Team 8331.
//
// SystemOrder::ARM chooses the form of the position loop when the controller is built. Geared 60:1, the NEO's back-EMF
// sets the arm's speed within about 16 ms, sooner than the 20 ms loop can act, so its position is first order, with its
// speed per volt as b0. A lightly geared arm would be second order. The ADRC's observer estimates gravity, friction and
// anything that bumps the arm as one disturbance and cancels it, so the arm needs no kG.
//
// With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
// /Tuning/ADRCArm/ArmMotor/closedloop/adrc.
class ADRCArmSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ADRCArmSubsystem();

  // Move the arm to an angle and hold it there.
  wpi::cmd::CommandPtr SetAngle(wpi::units::degree_t angle);

  // The arm's angle.
  wpi::units::degree_t GetAngle() const;

  // The arm's ADRC, built from its ADRCConfig by the motor controller; nullptr until it is configured.
  yams::math::ADRCController* GetADRC();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_armMotor{wpi::CANPort::CAN_S0, 30, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::ArmConfig m_armConfig;
  std::optional<yams::mechanisms::positional::Arm> m_arm;
};
