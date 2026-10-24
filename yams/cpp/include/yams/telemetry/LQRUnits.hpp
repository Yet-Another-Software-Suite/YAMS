// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <numbers>

#include "yams/math/LQRConfig.hpp"

namespace yams::telemetry {

/**
 * An LQR tolerance or standard deviation from the telemetry field's units to the SI units LQRConfig keeps: degrees to
 * radians and RPM to rad/s for an arm or flywheel; an elevator's meters and m/s are unchanged.
 *
 * @param type     LQRType of the LQR.
 * @param position True for a position, false for a velocity.
 * @param value    Value in the telemetry field's units.
 * @return Value in SI units.
 */
inline double LQRToSI(math::LQRConfig::LQRType type, bool position, double value) {
  if (type == math::LQRConfig::LQRType::ELEVATOR) return value;
  return position ? value * std::numbers::pi / 180.0 : value * 2.0 * std::numbers::pi / 60.0;
}

/**
 * An LQR tolerance or standard deviation from SI units to the telemetry field's units, the inverse of LQRToSI().
 *
 * @param type     LQRType of the LQR.
 * @param position True for a position, false for a velocity.
 * @param value    Value in SI units.
 * @return Value in the telemetry field's units.
 */
inline double LQRFromSI(math::LQRConfig::LQRType type, bool position, double value) {
  if (type == math::LQRConfig::LQRType::ELEVATOR) return value;
  return position ? value * 180.0 / std::numbers::pi : value * 60.0 / (2.0 * std::numbers::pi);
}

}  // namespace yams::telemetry
