// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/angle.h>

#include <ctre/phoenix6/TalonFXS.hpp>
#include <optional>

#include "yams/gearing/GearBox.hpp"
#include "yams/gearing/MechanismGearing.hpp"
#include "yams/mechanisms/config/PivotConfig.hpp"
#include "yams/mechanisms/positional/Pivot.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"

// Hood pivot driven by a TalonFXS (CAN 9) running a NEO motor through a 12:1 (3:1 x 4:1)
// reduction.  Position is controlled in degrees via MotionMagic with a trapezoid profile
// (0.5 tps max velocity, 0.25 tps^2 acceleration).  Soft limits: -30 deg to +100 deg.
// Hardware limits are enforced by the TalonFXS closed-loop controller, not by robot code.
//
// Commands exposed:
//   HoodCmd(dutycycle)   -- open-loop duty cycle, for tuning/override
//   SetAngle(angle)      -- closed-loop position command, runs until interrupted
class HoodSubsystem : public frc2::SubsystemBase {
 public:
  HoodSubsystem();

  // Direct setpoint write -- use when you need non-command periodic control.
  void SetAngleSetpoint(units::degree_t angle);

  frc2::CommandPtr HoodCmd(double dutycycle);        // open-loop override; runs continuously
  frc2::CommandPtr SetAngle(units::degree_t angle);  // closed-loop; ends when interrupted

  void Periodic() override;
  void SimulationPeriodic() override;

 private:
  ctre::phoenix6::hardware::TalonFXS m_hoodMotor{9};  // CAN ID 9

  yams::motorcontrollers::SmartMotorControllerConfig m_motorConfig;
  // optional because TalonFXSWrapper takes a pointer to m_hoodMotor, which must be
  // fully constructed before the wrapper is built; emplace() happens in the constructor.
  std::optional<yams::motorcontrollers::remote::TalonFXSWrapper> m_motor;

  yams::mechanisms::config::PivotConfig m_pivotConfig;  // angle limits: -100 to +200 deg
  std::optional<yams::mechanisms::positional::Pivot> m_hood;
};
