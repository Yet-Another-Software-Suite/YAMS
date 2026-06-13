// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace yams::gearing {

/**
 * Calculates input-to-output conversion factors for a multi-stage gearbox.
 *
 * Each stage is a ratio > 0 representing the speed reduction from input to output.
 * For example, a 3:1 reduction is expressed as 3.0.
 */
class GearBox {
 public:
  /**
   * Construct the GearBox with a single reduction stage.
   *
   * @param reductionStage Reduction stage where the value is > 0 to indicate a reduction.
   */
  explicit GearBox(double reductionStage);

  /**
   * Construct the GearBox with the reduction stages given.
   *
   * @param reductionStages Reduction stages where each value is > 0 to indicate a reduction.
   */
  explicit GearBox(std::initializer_list<double> reductionStages);

  /**
   * Construct the GearBox with the reduction stages given.
   *
   * @param reductionStages Reduction stages where each value is > 0 to indicate a reduction.
   */
  explicit GearBox(const std::vector<double>& reductionStages);

  /**
   * Construct the GearBox with the reduction stages given as "IN:OUT" strings.
   *
   * @param stages List of stages in the format "IN:OUT", e.g. "3:1".
   */
  explicit GearBox(const std::vector<std::string>& stages);

  /**
   * Create a GearBox from numeric reduction stages.
   *
   * @param stages Reduction stages where each value is > 0 to indicate a reduction.
   * @return GearBox.
   */
  static GearBox FromReductionStages(std::initializer_list<double> stages);

  /**
   * Create a GearBox from stages in "IN:OUT" string form.
   *
   * @param stages Stages in the format "IN:OUT", e.g. "3:1".
   * @return GearBox.
   */
  static GearBox FromStages(std::initializer_list<std::string_view> stages);

  /**
   * Create a GearBox from the tooth counts of each gear in the train.
   *
   * At least two gears (drive and driven) are required, and all tooth counts must be positive.
   *
   * @param teeth Gear tooth counts in mesh order.
   * @return GearBox.
   */
  static GearBox FromTeeth(std::initializer_list<int> teeth);

  /**
   * Multiply the gear reduction ratio by x.
   *
   * @param x Factor to multiply by.
   * @return *this for chaining.
   */
  GearBox& Times(double x);

  /**
   * Divide the gear reduction ratio by x.
   *
   * @param x Factor to divide by.
   * @return *this for chaining.
   */
  GearBox& Div(double x);

  /**
   * Get the conversion factor from gearbox input to gearbox output rotations (OUT/IN).
   *
   * @return OUT/IN conversion factor.
   */
  double GetInputToOutputConversionFactor() const;

  /**
   * Get the conversion factor from gearbox output to gearbox input rotations (IN/OUT).
   *
   * @return IN/OUT conversion factor.
   */
  double GetOutputToInputConversionFactor() const;

 private:
  std::vector<double> m_reductionStages;
  double m_gearReductionRatio{1.0};

  void SetupGearBox(const std::vector<double>& stages);
};

}  // namespace yams::gearing
