// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/Timer.h>
#include <units/time.h>

#include <cstdint>

namespace yams::math {

class DerivativeTimeFilter {
 public:
  DerivativeTimeFilter(double initial, units::second_t debouncerPeriod);
  explicit DerivativeTimeFilter(units::second_t debouncerPeriod);

  double Derivative(double current, units::second_t dt);
  double Derivative(double current);

 private:
  double m_last{0.0};
  int64_t m_lastFpgaTime_us{0};
  double m_value{0.0};
  frc::Timer m_debouncer;
  units::second_t m_debouncePeriod{0.02};
};

}  // namespace yams::math
