// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace yams::gearing {

/**
 * Calculates the conversion factor for a sprocket (chain/belt) stage chain.
 *
 * Each stage is a ratio > 0 representing the speed reduction from input to output.
 */
class Sprocket {
 public:
  /**
   * Create the Sprocket with a single reduction stage.
   *
   * @param reductionStage Reduction stage where the value is > 0 to indicate a reduction.
   */
  explicit Sprocket(double reductionStage);

  /**
   * Create the Sprocket with the reduction stages given.
   *
   * @param reductionStages Reduction stages where each value is > 0 to indicate a reduction.
   */
  explicit Sprocket(std::initializer_list<double> reductionStages);

  /**
   * Create the Sprocket with the reduction stages given.
   *
   * @param reductionStages Reduction stages where each value is > 0 to indicate a reduction.
   */
  explicit Sprocket(const std::vector<double>& reductionStages);

  /**
   * Create the Sprocket with the reduction stages given as "IN:OUT" strings.
   *
   * @param stages List of stages in the format "IN:OUT".
   */
  explicit Sprocket(const std::vector<std::string>& stages);

  /**
   * Create a Sprocket from stages in "IN:OUT" string form.
   *
   * @param stages Stages in the format "IN:OUT".
   * @return Sprocket.
   */
  static Sprocket FromStages(std::initializer_list<std::string_view> stages);

  /**
   * Multiply the sprocket reduction ratio by x.
   *
   * @param x Factor to multiply by.
   * @return *this for chaining.
   */
  Sprocket& Times(double x);

  /**
   * Divide the sprocket reduction ratio by x.
   *
   * @param x Factor to divide by.
   * @return *this for chaining.
   */
  Sprocket& Div(double x);

  /**
   * Get the conversion factor from sprocket input to sprocket output rotations (OUT/IN).
   *
   * @return OUT/IN conversion factor.
   */
  double GetInputToOutputConversionFactor() const;

  /**
   * Get the conversion factor from sprocket output to sprocket input rotations (IN/OUT).
   *
   * @return IN/OUT conversion factor.
   */
  double GetOutputToInputConversionFactor() const;

 private:
  std::vector<double> m_reductionStages;
  double m_sprocketReductionRatio{1.0};

  void SetupStages(const std::vector<double>& stages);
};

}  // namespace yams::gearing
