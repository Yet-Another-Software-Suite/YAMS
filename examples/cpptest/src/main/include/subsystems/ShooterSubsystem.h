// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/math/system/DCMotor.hpp>
#include <wpi/commands2/CommandPtr.hpp>
#include <wpi/commands2/SubsystemBase.hpp>
#include <rev/SparkMax.h>
#include <wpi/hardware/bus/CANPort.hpp>
#include <wpi/units/angular_velocity.hpp>
#include <wpi/units/time.hpp>
#include <wpi/units/velocity.hpp>
#include <wpi/units/voltage.hpp>

#include <functional>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/FlyWheelConfig.hpp"
#include "yams/mechanisms/velocity/FlyWheel.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"

// Two REV NEO motors on CAN IDs 3 (leader) and 4 (follower), driving a single
// 4-inch compliant shooter wheel. No gearbox between the motor shaft and wheel
// shaft -- the 12:1 reduction in the config is notional for velocity scaling.
//
// Motor 2 (CAN 4) is registered as a hardware follower of motor 1 via SparkWrapper's
// tight-coupling mechanism; it mirrors every output without software intervention.
//
// Exposed commands:
//   SetVelocity(deg/s or lambda) -- closed-loop velocity via FlyWheel::Run
//   Set(dutyCycle or lambda)     -- open-loop percent output
//   ReadyToShoot(tolerance)      -- boolean check for a "at speed" gate
//
// Setpoint-only variants (SetVelocitySetpoint, SetDutyCycleSetpoint,
// SetSurfaceSpeedSetpoint) write a setpoint directly without wrapping in a
// Command -- useful inside a larger sequential command.
class ShooterSubsystem : public wpi::cmd::SubsystemBase {
 public:
  ShooterSubsystem();

  wpi::units::degrees_per_second_t GetVelocity() const;
  bool ReadyToShoot(wpi::units::degrees_per_second_t tolerance) const;

  wpi::cmd::CommandPtr SetVelocity(wpi::units::degrees_per_second_t speed);
  wpi::cmd::CommandPtr SetVelocity(std::function<wpi::units::degrees_per_second_t()> speed);
  wpi::cmd::CommandPtr Set(double dutyCycle);
  wpi::cmd::CommandPtr Set(std::function<double()> dutyCycle);

  // Write a setpoint directly; caller is responsible for scheduling/requirements.
  void SetVelocitySetpoint(wpi::units::degrees_per_second_t speed);
  void SetDutyCycleSetpoint(double dutyCycle);
  void SetSurfaceSpeedSetpoint(wpi::units::meters_per_second_t speed);

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  // Both motors must be declared before m_motorConfig so their addresses are
  // stable when WithFollowers stores &m_flywheelMotor2 in std::any.
  rev::spark::SparkMax m_flywheelMotor1{wpi::CANPort::CAN_S0, 3, rev::spark::SparkMax::MotorType::kBrushless};
  rev::spark::SparkMax m_flywheelMotor2{wpi::CANPort::CAN_S0, 4, rev::spark::SparkMax::MotorType::kBrushless};

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  // SparkWrapper constructed in .cpp after config is fully built; std::optional
  // avoids default-constructing before the hardware object is ready.
  std::optional<yams::motorcontrollers::local::SparkWrapper> m_motor;

  yams::mechanisms::config::FlyWheelConfig m_shooterConfig;
  std::optional<yams::mechanisms::velocity::FlyWheel> m_shooter;
};
