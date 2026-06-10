// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when an ArmConfig is incomplete or missing required simulation fields.
 */
class ArmConfigurationException : public YamsException {
 public:
  /**
   * Construct an ArmConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  ArmConfigurationException(const std::string& issue, const std::string& result,
                             const std::string& fix)
      : YamsException(
            "[Arm Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams
