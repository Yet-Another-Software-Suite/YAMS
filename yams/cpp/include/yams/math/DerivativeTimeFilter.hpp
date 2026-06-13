// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <frc/Timer.h>
#include <units/time.h>

#include <cstdint>

namespace yams::math {

/**
 * Computes the time-derivative of a scalar value, debounced to avoid updating too frequently.
 *
 * The derivative is computed over the elapsed FPGA time between calls and is gated by a
 * configurable debounce period so high-frequency callers still receive stable readings.
 */
class DerivativeTimeFilter {
 public:
  /**
   * Create a derivative filter with a known initial value.
   *
   * The initial value is timestamped at the time of construction.
   *
   * @param initial         Initial value to differentiate from.
   * @param debouncerPeriod Minimum period between derivative updates.
   */
  DerivativeTimeFilter(double initial, units::second_t debouncerPeriod);

  /**
   * Create a derivative filter with no initial value (starts from 0).
   *
   * @param debouncerPeriod Minimum period between derivative updates.
   */
  explicit DerivativeTimeFilter(units::second_t debouncerPeriod);

  /**
   * Get the derivative of the current value over a specified delta time.
   *
   * If the debounce period has not elapsed, returns the cached result.
   *
   * @param current Current value.
   * @param dt      Delta time to use as the denominator.
   * @return Derivative of current from the previous value divided by dt.
   */
  double Derivative(double current, units::second_t dt);

  /**
   * Get the derivative of the current value over the time elapsed since the last call.
   *
   * If the debounce period has not elapsed, returns the cached result.
   *
   * @param current Current value.
   * @return Derivative of current from the previous value over the elapsed FPGA time.
   */
  double Derivative(double current);

 private:
  double m_last{0.0};
  int64_t m_lastFpgaTime_us{0};
  double m_value{0.0};
  frc::Timer m_debouncer;
  units::second_t m_debouncePeriod{0.02};
};

}  // namespace yams::math
