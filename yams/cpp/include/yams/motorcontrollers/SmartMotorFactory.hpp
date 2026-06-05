// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/system/plant/DCMotor.h>
#include <rev/SparkFlex.h>
#include <rev/SparkMax.h>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/TalonFXS.hpp>
#include <memory>

#include "yams/motorcontrollers/SmartMotorController.hpp"
#include "yams/motorcontrollers/SmartMotorControllerConfig.hpp"
#include "yams/motorcontrollers/local/SparkWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.hpp"
#include "yams/motorcontrollers/remote/TalonFXWrapper.hpp"

namespace yams::motorcontrollers {

/**
 * Factory class for constructing SmartMotorController instances.
 *
 * All construction methods are static; this class cannot be instantiated.
 */
class SmartMotorFactory {
 public:
  SmartMotorFactory() = delete;

  /**
   * Create a SmartMotorController wrapping a CTRE TalonFX.
   *
   * @param talon  TalonFX hardware object (passed by reference; must outlive the wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config SmartMotorControllerConfig to apply.
   * @return Owning pointer to the constructed TalonFXWrapper.
   */
  static std::unique_ptr<SmartMotorController> Create(ctre::phoenix6::hardware::TalonFX& talon,
                                                      frc::DCMotor motor,
                                                      const SmartMotorControllerConfig& config);

  /**
   * Create a SmartMotorController wrapping a CTRE TalonFXS.
   *
   * @param talon       TalonFXS hardware object (passed by reference; must outlive the wrapper).
   * @param motor       DC motor model used for simulation.
   * @param arrangement Motor arrangement enum describing the attached motor type.
   * @param config      SmartMotorControllerConfig to apply.
   * @return Owning pointer to the constructed TalonFXSWrapper.
   */
  static std::unique_ptr<SmartMotorController> Create(
      ctre::phoenix6::hardware::TalonFXS& talon, frc::DCMotor motor,
      remote::TalonFXSWrapper::MotorArrangement arrangement,
      const SmartMotorControllerConfig& config);

  /**
   * Create a SmartMotorController wrapping a REV SparkMax.
   *
   * @param spark  SparkMax hardware object (passed by reference; must outlive the wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config SmartMotorControllerConfig to apply.
   * @return Owning pointer to the constructed SparkWrapper.
   */
  static std::unique_ptr<SmartMotorController> Create(rev::spark::SparkMax& spark,
                                                      frc::DCMotor motor,
                                                      const SmartMotorControllerConfig& config);

  /**
   * Create a SmartMotorController wrapping a REV SparkFlex.
   *
   * @param spark  SparkFlex hardware object (passed by reference; must outlive the wrapper).
   * @param motor  DC motor model used for simulation.
   * @param config SmartMotorControllerConfig to apply.
   * @return Owning pointer to the constructed SparkWrapper.
   */
  static std::unique_ptr<SmartMotorController> Create(rev::spark::SparkFlex& spark,
                                                      frc::DCMotor motor,
                                                      const SmartMotorControllerConfig& config);
};

}  // namespace yams::motorcontrollers
