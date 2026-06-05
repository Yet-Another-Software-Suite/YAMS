// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/SmartMath.hpp"

namespace yams::math {

double SmartMath::SensorToMechanismRatio(std::initializer_list<double> stages) {
  if (stages.size() == 0) {
    throw NoStagesGivenException{};
  }
  double ratio = 1.0;
  for (double s : stages) {
    ratio *= s;
  }
  return ratio;
}

double SmartMath::GearBox(std::initializer_list<double> stages) {
  if (stages.size() == 0) {
    throw NoStagesGivenException{};
  }
  double result = 1.0;
  for (double s : stages) {
    result *= s;
  }
  return result;
}

}  // namespace yams::math
