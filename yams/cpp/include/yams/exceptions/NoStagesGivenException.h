// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include "YamsException.h"

namespace yams {

class NoStagesGivenException : public YamsException {
 public:
  NoStagesGivenException()
      : YamsException("No reduction stages were given to the GearBox/Sprocket!") {}
};

}  // namespace yams
