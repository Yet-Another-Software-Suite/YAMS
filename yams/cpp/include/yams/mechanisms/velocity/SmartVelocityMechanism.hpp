// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <frc2/command/button/Trigger.h>

#include "yams/mechanisms/SmartMechanism.hpp"

namespace yams::mechanisms::velocity {

/**
 * Abstract base class for velocity-controlled mechanisms (flywheels, rollers,
 * etc.).
 *
 * Extends SmartMechanism with hardware limit triggers and Mechanism2d
 * ligament/root accessors for visualisation.
 */
class SmartVelocityMechanism : public SmartMechanism {
 public:
  SmartVelocityMechanism() : SmartMechanism() {}
  virtual ~SmartVelocityMechanism() = default;

  // ---- Pure virtual interface -----------------------------------------------

  /**
   * Trigger that becomes true when the mechanism is at or past its maximum
   * velocity (or forward / upper) limit.
   *
   * @return Trigger for the maximum limit condition.
   */
  virtual frc2::Trigger Max() = 0;

  /**
   * Trigger that becomes true when the mechanism is at or past its minimum
   * velocity (or reverse / lower) limit.
   *
   * @return Trigger for the minimum limit condition.
   */
  virtual frc2::Trigger Min() = 0;

  // ---- Visualisation accessors ----------------------------------------------

  /**
   * Get the MechanismLigament2d used to animate this mechanism.
   *
   * @return Pointer to the ligament, or nullptr if not initialised.
   */
  frc::MechanismLigament2d* GetMechanismLigament();

  /**
   * Get the MechanismRoot2d anchor point for this mechanism.
   *
   * @return Pointer to the root, or nullptr if not initialised.
   */
  frc::MechanismRoot2d* GetMechanismRoot();

  // ---- Motor accessor -------------------------------------------------------

  /**
   * Get the underlying SmartMotorController (alias for GetMotorController()).
   *
   * @return Pointer to the motor controller.
   */
  motorcontrollers::SmartMotorController* GetMotor();

 protected:
  /** Root anchor point in the Mechanism2d canvas. */
  frc::MechanismRoot2d* m_mechanismRoot{nullptr};

  /** Animated ligament representing the spinning element of the mechanism. */
  frc::MechanismLigament2d* m_mechanismLigament{nullptr};
};

}  // namespace yams::mechanisms::velocity
