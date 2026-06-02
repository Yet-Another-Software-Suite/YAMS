// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <stdexcept>
#include <string>

namespace yams {

class YamsException : public std::runtime_error {
 public:
  explicit YamsException(const std::string& message) : std::runtime_error(message) {}
};

}  // namespace yams
