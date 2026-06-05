// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <string>

#include "YamsException.hpp"

namespace yams {

/**
 * Thrown when a SwerveDriveConfig is invalid or incomplete.
 */
class SwerveDriveConfigurationException : public YamsException {
 public:
  /**
   * Construct a SwerveDriveConfigurationException.
   *
   * @param message Description of the configuration error.
   */
  explicit SwerveDriveConfigurationException(const std::string& message)
      : YamsException("[SwerveDrive Configuration Error]\n" + message) {}
};

}  // namespace yams
