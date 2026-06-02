// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>

#include <optional>

#include "EasyCRTConfig.h"

namespace yams::units {

class EasyCRT {
 public:
  enum class CRTStatus {
    OK,
    NO_SOLUTION,
    AMBIGUOUS,
    NOT_ATTEMPTED,
    INVALID_CONFIG,
  };

  explicit EasyCRT(EasyCRTConfig config);

  std::optional<::units::angle::turn_t> GetAngleOptional();

  CRTStatus GetLastStatus() const;
  double GetLastErrorRotations() const;
  int GetLastIterations() const;

 private:
  struct CrtSolution {
    double mechanismRotations;
    double errorRotations;
  };

  EasyCRTConfig m_config;
  CRTStatus m_lastStatus{CRTStatus::NOT_ATTEMPTED};
  double m_lastErrorRot{std::numeric_limits<double>::quiet_NaN()};
  int m_lastIterations{0};

  std::optional<CrtSolution> ResolveFromSensors(double abs1, double abs2, double ratio1,
                                                double ratio2, double minMechRot, double maxMechRot,
                                                double matchTolerance);

  static double ModularError(double a, double b);
};

}  // namespace yams::units
