// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace yams::gearing {

class Sprocket {
 public:
  explicit Sprocket(double reductionStage);
  explicit Sprocket(std::initializer_list<double> reductionStages);
  explicit Sprocket(const std::vector<double>& reductionStages);
  explicit Sprocket(const std::vector<std::string>& stages);

  static Sprocket FromStages(std::initializer_list<std::string_view> stages);

  Sprocket& Times(double x);
  Sprocket& Div(double x);

  double GetInputToOutputConversionFactor() const;
  double GetOutputToInputConversionFactor() const;

 private:
  std::vector<double> m_reductionStages;
  double m_sprocketReductionRatio{1.0};

  void SetupStages(const std::vector<double>& stages);
};

}  // namespace yams::gearing
