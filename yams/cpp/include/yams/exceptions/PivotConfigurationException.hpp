// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when a PivotConfig is incomplete or missing required simulation fields.
 */
class PivotConfigurationException : public YamsException {
 public:
  /**
   * Construct a PivotConfigurationException.
   *
   * @param issue  Short description of what is misconfigured.
   * @param result Description of the undesirable consequence.
   * @param fix    Suggested remediation for the caller.
   */
  PivotConfigurationException(const std::string& issue, const std::string& result,
                               const std::string& fix)
      : YamsException(
            "[Pivot Configuration Error]\n"
            "Issue: " +
            issue + "\nResult: " + result + "\nFix: " + fix) {}
};

}  // namespace yams
