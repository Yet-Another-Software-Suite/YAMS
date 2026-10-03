// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <rev/config/AbsoluteEncoderConfig.h>
#include <rev/config/ClosedLoopConfig.h>
#include <rev/config/EncoderConfig.h>

namespace yams::motorcontrollers::local {

/**
 * REVLib 2027.0.0-alpha-7 removed the typed setters for encoder conversion factors and
 * closed-loop position-wrapping input range from rev::spark::EncoderConfig,
 * rev::spark::AbsoluteEncoderConfig, and rev::spark::ClosedLoopConfig; but the underlying raw CAN
 * parameters (unchanged since 2026 REVLib's SparkParameters table) still work. These subclasses
 * restore them by writing those parameter IDs directly through the protected PutParameter()
 * REVLib exposes to config subclasses, then get merged onto the real config object via its
 * public Apply(...).
 */
class RevEncoderConversionFactors : public rev::spark::EncoderConfig {
 public:
  RevEncoderConversionFactors& WithPositionConversionFactor(double factor) {
    PutParameter(kPositionConversionFactor, static_cast<float>(factor));
    return *this;
  }

  RevEncoderConversionFactors& WithVelocityConversionFactor(double factor) {
    PutParameter(kVelocityConversionFactor, static_cast<float>(factor));
    return *this;
  }

 private:
  static constexpr uint8_t kPositionConversionFactor = 112;
  static constexpr uint8_t kVelocityConversionFactor = 113;
};

/** @see RevEncoderConversionFactors */
class RevAbsoluteEncoderConversionFactors : public rev::spark::AbsoluteEncoderConfig {
 public:
  RevAbsoluteEncoderConversionFactors() { SetSparkMaxDataPortConfig(); }

  RevAbsoluteEncoderConversionFactors& WithPositionConversionFactor(double factor) {
    PutParameter(kDutyCyclePositionFactor, static_cast<float>(factor));
    return *this;
  }

  RevAbsoluteEncoderConversionFactors& WithVelocityConversionFactor(double factor) {
    PutParameter(kDutyCycleVelocityFactor, static_cast<float>(factor));
    return *this;
  }

 private:
  static constexpr uint8_t kDutyCyclePositionFactor = 139;
  static constexpr uint8_t kDutyCycleVelocityFactor = 140;
};

/** @see RevEncoderConversionFactors */
class RevClosedLoopPositionWrapping : public rev::spark::ClosedLoopConfig {
 public:
  RevClosedLoopPositionWrapping& WithMinInput(double minInput) {
    PutParameter(kPositionPidMinInput, static_cast<float>(minInput));
    return *this;
  }

  RevClosedLoopPositionWrapping& WithMaxInput(double maxInput) {
    PutParameter(kPositionPidMaxInput, static_cast<float>(maxInput));
    return *this;
  }

 private:
  static constexpr uint8_t kPositionPidMinInput = 150;
  static constexpr uint8_t kPositionPidMaxInput = 151;
};

}  // namespace yams::motorcontrollers::local
