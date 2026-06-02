// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <stdexcept>

#include "yams/exceptions/NoStagesGivenException.h"

namespace yams::math {

class SmartMath {
 public:
  SmartMath() = delete;

  static double SensorToMechanismRatio(std::initializer_list<double> stages);
  static double GearBox(std::initializer_list<double> stages);
};

}  // namespace yams::math
