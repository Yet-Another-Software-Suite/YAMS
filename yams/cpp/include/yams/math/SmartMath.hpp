// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <stdexcept>

#include "yams/exceptions.hpp"

namespace yams::math {

/**
 * Utility class providing static helper methods for gearing calculations.
 *
 * Cannot be instantiated.
 */
class SmartMath {
 public:
  SmartMath() = delete;

  /**
   * Compute the sensor-to-mechanism ratio from a list of reduction stages.
   *
   * Each stage value is > 0, where a value greater than 1 indicates a speed reduction.
   *
   * @param stages Reduction stages (e.g. {10.0, 2.0} for a 20:1 total reduction).
   * @return Combined sensor-to-mechanism conversion factor.
   */
  static double SensorToMechanismRatio(std::initializer_list<double> stages);

  /**
   * Compute the overall gear reduction ratio from a list of reduction stages.
   *
   * Equivalent to SensorToMechanismRatio but named explicitly for gearbox calculations.
   *
   * @param stages Reduction stages.
   * @return Combined gear reduction factor.
   */
  static double GearBox(std::initializer_list<double> stages);
};

}  // namespace yams::math
