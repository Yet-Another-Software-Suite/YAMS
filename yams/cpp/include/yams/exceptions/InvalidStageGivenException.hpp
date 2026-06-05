// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when a gearbox or sprocket stage string is not in the required "IN:OUT" format.
 */
class InvalidStageGivenException : public YamsException {
 public:
  /**
   * Construct an InvalidStageGivenException for the offending stage string.
   *
   * @param stage The invalid stage string that was provided.
   */
  explicit InvalidStageGivenException(const std::string& stage)
      : YamsException("Invalid stage given: \"" + stage +
                      "\". Stage must be in the format of \"IN:OUT\"") {}
};

}  // namespace yams
