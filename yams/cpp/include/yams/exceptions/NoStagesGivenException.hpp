// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when a GearBox or Sprocket is constructed with an empty list of reduction stages.
 */
class NoStagesGivenException : public YamsException {
 public:
  /** Construct a NoStagesGivenException. */
  NoStagesGivenException()
      : YamsException("No reduction stages were given to the GearBox/Sprocket!") {}
};

}  // namespace yams
