// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.h"

namespace yams {

class SmartMotorControllerConfigurationException : public YamsException {
 public:
  SmartMotorControllerConfigurationException(const std::string& issue, const std::string& result,
                                             const std::string& fix)
      : YamsException(
            "[SmartMotorController Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams
