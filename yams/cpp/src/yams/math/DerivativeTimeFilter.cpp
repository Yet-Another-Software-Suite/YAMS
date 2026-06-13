// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/math/DerivativeTimeFilter.hpp"

#include <frc/RobotController.h>
#include <units/time.h>

namespace yams::math {

DerivativeTimeFilter::DerivativeTimeFilter(double initial, units::second_t debouncerPeriod)
    : m_last(initial),
      m_lastFpgaTime_us(frc::RobotController::GetFPGATime()),
      m_debouncePeriod(debouncerPeriod) {
  m_debouncer.Start();
}

DerivativeTimeFilter::DerivativeTimeFilter(units::second_t debouncerPeriod)
    : m_debouncePeriod(debouncerPeriod) {
  m_debouncer.Start();
}

double DerivativeTimeFilter::Derivative(double current, units::second_t dt) {
  if (m_debouncer.AdvanceIfElapsed(m_debouncePeriod)) {
    double derivative = (current - m_last) / dt.value();
    m_last = current;
    m_value = derivative;
    return derivative;
  }
  return m_value;
}

double DerivativeTimeFilter::Derivative(double current) {
  if (m_debouncer.HasElapsed(m_debouncePeriod)) {
    int64_t currentFpgaTime_us = frc::RobotController::GetFPGATime();
    double deriv = Derivative(
        current, units::microsecond_t{static_cast<double>(currentFpgaTime_us - m_lastFpgaTime_us)});
    m_lastFpgaTime_us = currentFpgaTime_us;
    return deriv;
  }
  return m_value;
}

}  // namespace yams::math
