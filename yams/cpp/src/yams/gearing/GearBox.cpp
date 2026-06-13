// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/gearing/GearBox.hpp"

#include <stdexcept>
#include <string>
#include <vector>

#include "yams/exceptions.hpp"

namespace yams::gearing {

GearBox::GearBox(double reductionStage) { SetupGearBox({reductionStage}); }

GearBox::GearBox(std::initializer_list<double> reductionStages) {
  SetupGearBox(std::vector<double>(reductionStages));
}

GearBox::GearBox(const std::vector<double>& reductionStages) { SetupGearBox(reductionStages); }

GearBox::GearBox(const std::vector<std::string>& stages) {
  std::vector<double> ratios;
  ratios.reserve(stages.size());
  for (const auto& stage : stages) {
    auto colon = stage.find(':');
    if (colon == std::string::npos) {
      throw exceptions::InvalidStageGivenException(stage);
    }
    double in = std::stod(stage.substr(0, colon));
    double out = std::stod(stage.substr(colon + 1));
    ratios.push_back(in / out);
  }
  SetupGearBox(ratios);
}

GearBox GearBox::FromReductionStages(std::initializer_list<double> stages) {
  return GearBox(stages);
}

GearBox GearBox::FromStages(std::initializer_list<std::string_view> stages) {
  std::vector<std::string> v;
  v.reserve(stages.size());
  for (auto s : stages) {
    v.emplace_back(s);
  }
  return GearBox(v);
}

GearBox GearBox::FromTeeth(std::initializer_list<int> teeth) {
  if (teeth.size() < 2) {
    throw std::invalid_argument("At least two gears (drive and driven) are required");
  }
  double ratio = 1.0;
  auto it = teeth.begin();
  int prev = *it++;
  while (it != teeth.end()) {
    int curr = *it++;
    if (prev <= 0 || curr <= 0) {
      throw std::invalid_argument("Gear teeth counts must be positive integers");
    }
    ratio *= static_cast<double>(curr) / static_cast<double>(prev);
    prev = curr;
  }
  return GearBox(ratio);
}

GearBox& GearBox::Times(double x) {
  m_gearReductionRatio *= x;
  return *this;
}

GearBox& GearBox::Div(double x) {
  m_gearReductionRatio /= x;
  return *this;
}

double GearBox::GetInputToOutputConversionFactor() const { return m_gearReductionRatio; }

double GearBox::GetOutputToInputConversionFactor() const { return 1.0 / m_gearReductionRatio; }

void GearBox::SetupGearBox(const std::vector<double>& stages) {
  if (stages.empty()) {
    throw exceptions::NoStagesGivenException{};
  }
  m_reductionStages = stages;
  double gearBox = 1.0 / stages[0];
  for (size_t i = 1; i < stages.size(); ++i) {
    gearBox *= (1.0 / stages[i]);
  }
  m_gearReductionRatio = gearBox;
}

}  // namespace yams::gearing
