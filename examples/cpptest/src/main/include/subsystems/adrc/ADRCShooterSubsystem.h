// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <rev/SparkMax.h>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/units/angular_velocity.hpp>

#include <optional>

#include "yams/math/ADRCController.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// A shooter flywheel run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO
// (CAN 32) turning a 4 in, 1 lb wheel directly. The ADRC was written by FRC Team 8331.
//
// A flywheel's velocity is a first order plant: a voltage accelerates it. The ADRC's observer estimates the back-EMF,
// friction and the speed a game piece takes out of the wheel as one disturbance and cancels it, so the shooter needs
// no kS or kV and recovers quickly after each shot.
//
// With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
// /Tuning/ADRCShooter/ShooterMotor/closedloop/adrc.
class ADRCShooterSubsystem : public wpi::cmd::SubsystemBase {
 public:
  // Coasts whenever nothing else is using the shooter.
  ADRCShooterSubsystem();

  // Spin the shooter at a speed until interrupted. Starts the closed loop controller, which runs the ADRC on the
  // SystemCore, if coasting stopped it.
  wpi::cmd::CommandPtr SpinUp(wpi::units::degrees_per_second_t speed);

  // The shooter's speed.
  wpi::units::degrees_per_second_t GetSpeed() const;

  // The shooter's ADRC, built from its ADRCConfig by the motor controller; nullptr until it is configured.
  yams::math::ADRCController* GetADRC();

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  rev::spark::SparkMax m_shooterMotor{wpi::CANPort::CAN_S0, 32, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::FlyWheelConfig m_shooterConfig;
  std::optional<yams::mechanisms::velocity::FlyWheel> m_shooter;
};
