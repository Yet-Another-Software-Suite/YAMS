// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <stdexcept>
#include <string>

namespace yams {

/**
 * Base exception class for all YAMS runtime errors.
 *
 * All YAMS exceptions derive from this class, which itself extends std::runtime_error.
 */
class YamsException : public std::runtime_error {
 public:
  /**
   * Construct a YamsException with the given message.
   *
   * @param message Human-readable description of the error.
   */
  explicit YamsException(const std::string& message) : std::runtime_error(message) {}
};

}  // namespace yams
