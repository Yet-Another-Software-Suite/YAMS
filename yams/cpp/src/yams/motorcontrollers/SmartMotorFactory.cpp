// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/motorcontrollers/SmartMotorFactory.h"

#include <memory>

#include "yams/motorcontrollers/local/SparkWrapper.h"
#include "yams/motorcontrollers/remote/TalonFXSWrapper.h"
#include "yams/motorcontrollers/remote/TalonFXWrapper.h"

namespace yams::motorcontrollers {

std::unique_ptr<SmartMotorController> SmartMotorFactory::Create(
    ctre::phoenix6::hardware::TalonFX& talon, frc::DCMotor motor,
    const SmartMotorControllerConfig& config) {
  return std::make_unique<remote::TalonFXWrapper>(talon, motor, config);
}

std::unique_ptr<SmartMotorController> SmartMotorFactory::Create(
    ctre::phoenix6::hardware::TalonFXS& talon, frc::DCMotor motor,
    remote::TalonFXSWrapper::MotorArrangement arrangement,
    const SmartMotorControllerConfig& config) {
  return std::make_unique<remote::TalonFXSWrapper>(talon, motor, arrangement, config);
}

std::unique_ptr<SmartMotorController> SmartMotorFactory::Create(
    rev::spark::SparkMax& spark, frc::DCMotor motor, const SmartMotorControllerConfig& config) {
  return std::make_unique<local::SparkWrapper>(spark, motor, config);
}

std::unique_ptr<SmartMotorController> SmartMotorFactory::Create(
    rev::spark::SparkFlex& spark, frc::DCMotor motor, const SmartMotorControllerConfig& config) {
  return std::make_unique<local::SparkWrapper>(spark, motor, config);
}

}  // namespace yams::motorcontrollers
