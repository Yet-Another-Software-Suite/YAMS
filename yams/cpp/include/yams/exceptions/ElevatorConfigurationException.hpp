// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when an ElevatorConfig is incomplete or missing required simulation fields.
 */
class ElevatorConfigurationException : public YamsException {
 public:
  /**
   * Construct an ElevatorConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  ElevatorConfigurationException(const std::string& issue, const std::string& result,
                                 const std::string& fix)
      : YamsException(
            "[Elevator Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams
