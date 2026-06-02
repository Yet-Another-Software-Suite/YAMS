// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <initializer_list>
#include <string>
#include <string_view>
#include <vector>

namespace yams::gearing {

class GearBox {
 public:
  explicit GearBox(double reductionStage);
  explicit GearBox(std::initializer_list<double> reductionStages);
  explicit GearBox(const std::vector<double>& reductionStages);
  explicit GearBox(const std::vector<std::string>& stages);

  static GearBox FromReductionStages(std::initializer_list<double> stages);
  static GearBox FromStages(std::initializer_list<std::string_view> stages);
  static GearBox FromTeeth(std::initializer_list<int> teeth);

  GearBox& Times(double x);
  GearBox& Div(double x);

  double GetInputToOutputConversionFactor() const;
  double GetOutputToInputConversionFactor() const;

 private:
  std::vector<double> m_reductionStages;
  double m_gearReductionRatio{1.0};

  void SetupGearBox(const std::vector<double>& stages);
};

}  // namespace yams::gearing
