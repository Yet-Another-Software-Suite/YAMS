// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.h"

namespace yams {

class InvalidStageGivenException : public YamsException {
 public:
  explicit InvalidStageGivenException(const std::string& stage)
      : YamsException("Invalid stage given: \"" + stage +
                      "\". Stage must be in the format of \"IN:OUT\"") {}
};

}  // namespace yams
