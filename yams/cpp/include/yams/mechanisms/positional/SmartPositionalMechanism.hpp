// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/smartdashboard/MechanismLigament2d.hpp>
#include <wpi/smartdashboard/MechanismRoot2d.hpp>
#include <wpi/commands2/button/Trigger.hpp>

#include "yams/mechanisms/SmartMechanism.hpp"

namespace yams::mechanisms::positional {

/**
 * Abstract base class for positionally-controlled mechanisms (arms, elevators,
 * pivots, etc.).
 *
 * Extends SmartMechanism with hardware limit triggers and Mechanism2d
 * ligament/root accessors for visualisation.
 */
class SmartPositionalMechanism : public SmartMechanism {
 public:
  SmartPositionalMechanism() : SmartMechanism() {}
  virtual ~SmartPositionalMechanism() = default;

  // ---- Pure virtual interface -----------------------------------------------

  /**
   * Trigger that becomes true when the mechanism is at or past its maximum
   * (forward / upper) hard limit.
   *
   * @return Trigger for the maximum limit condition.
   */
  virtual wpi::cmd::Trigger Max() = 0;

  /**
   * Trigger that becomes true when the mechanism is at or past its minimum
   * (reverse / lower) hard limit.
   *
   * @return Trigger for the minimum limit condition.
   */
  virtual wpi::cmd::Trigger Min() = 0;

  // ---- Visualisation accessors ----------------------------------------------

  /**
   * Get the MechanismLigament2d used to animate this mechanism.
   *
   * @return Pointer to the ligament, or nullptr if not initialised.
   */
  wpi::MechanismLigament2d* GetMechanismLigament();

  /**
   * Get the MechanismRoot2d anchor point for this mechanism.
   *
   * @return Pointer to the root, or nullptr if not initialised.
   */
  wpi::MechanismRoot2d* GetMechanismRoot();

  // ---- Motor accessor -------------------------------------------------------

  /**
   * Get the underlying SmartMotorController (alias for GetMotorController()).
   *
   * @return Pointer to the motor controller.
   */
  motorcontrollers::SmartMotorController* GetMotor();

 protected:
  /** Root anchor point in the Mechanism2d canvas. */
  wpi::MechanismRoot2d* m_mechanismRoot{nullptr};

  /** Animated ligament representing the moving part of the mechanism. */
  wpi::MechanismLigament2d* m_mechanismLigament{nullptr};
};

}  // namespace yams::mechanisms::positional
