// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.h"

namespace yams {

/**
 * Thrown when a SmartMotorControllerConfig is incomplete or contains conflicting settings.
 */
class SmartMotorControllerConfigurationException : public YamsException {
 public:
  /**
   * Construct a SmartMotorControllerConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  SmartMotorControllerConfigurationException(const std::string& issue, const std::string& result,
                                             const std::string& fix)
      : YamsException(
            "[SmartMotorController Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams
