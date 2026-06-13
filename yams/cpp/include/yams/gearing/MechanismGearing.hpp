// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>

#include "GearBox.hpp"
#include "Sprocket.hpp"

namespace yams::gearing {

/**
 * Mechanism gearing for conversions from the motor rotor to the mechanism output.
 *
 * Combines an optional GearBox and an optional Sprocket chain to produce a single
 * rotor-to-mechanism ratio used by the motor controller configuration.
 */
class MechanismGearing {
 public:
  /** 1:1 MechanismGearing (no reduction). */
  static const MechanismGearing kOne;

  /**
   * Construct a MechanismGearing with a single reduction ratio.
   *
   * @param reductionRatio Reduction ratio (e.g. 3.0 for 3:1, 0.5 for 1:2).
   */
  explicit MechanismGearing(double reductionRatio);

  /**
   * Construct a MechanismGearing with only a GearBox attached to the motor.
   *
   * @param gearBox GearBox of the mechanism.
   */
  explicit MechanismGearing(const GearBox& gearBox);

  /**
   * Construct a MechanismGearing with a GearBox and a downstream Sprocket chain.
   *
   * @param gearBox   GearBox attached to the motor.
   * @param sprockets Sprocket chain attached to the gearbox output.
   */
  MechanismGearing(const GearBox& gearBox, const Sprocket& sprockets);

  /**
   * Get the rotor-to-mechanism ratio (OUT/IN).
   *
   * @return OUT/IN ratio to use for sensor-to-mechanism calculations.
   */
  double GetRotorToMechanismRatio() const;

  /**
   * Get the mechanism-to-rotor ratio (IN/OUT), i.e. the overall reduction.
   *
   * @return IN/OUT ratio to use for mechanism-to-sensor calculations.
   */
  double GetMechanismToRotorRatio() const;

  /**
   * Divide the gearbox reduction ratio by i.
   *
   * @param i Divisor.
   * @return *this for chaining.
   */
  MechanismGearing& Div(double i);

 private:
  GearBox m_gearBox;
  std::optional<Sprocket> m_sprockets;
};

}  // namespace yams::gearing
