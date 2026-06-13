// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/gearing/Sprocket.hpp"

#include <string>
#include <vector>

#include "yams/exceptions.hpp"

namespace yams::gearing {
Sprocket::Sprocket(double reductionStage) { SetupStages({reductionStage}); }

Sprocket::Sprocket(std::initializer_list<double> reductionStages) {
  SetupStages(std::vector<double>(reductionStages));
}

Sprocket::Sprocket(const std::vector<double>& reductionStages) { SetupStages(reductionStages); }

Sprocket::Sprocket(const std::vector<std::string>& stages) {
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
  SetupStages(ratios);
}

Sprocket Sprocket::FromStages(std::initializer_list<std::string_view> stages) {
  std::vector<std::string> v;
  v.reserve(stages.size());
  for (auto s : stages) {
    v.emplace_back(s);
  }
  return Sprocket(v);
}

Sprocket& Sprocket::Times(double x) {
  m_sprocketReductionRatio *= x;
  return *this;
}

Sprocket& Sprocket::Div(double x) {
  m_sprocketReductionRatio /= x;
  return *this;
}

double Sprocket::GetInputToOutputConversionFactor() const { return 1.0 / m_sprocketReductionRatio; }

double Sprocket::GetOutputToInputConversionFactor() const { return m_sprocketReductionRatio; }

void Sprocket::SetupStages(const std::vector<double>& stages) {
  if (stages.empty()) {
    throw exceptions::NoStagesGivenException{};
  }
  m_reductionStages = stages;
  double ratio = 1.0 / stages[0];
  for (size_t i = 1; i < stages.size(); ++i) {
    ratio *= (1.0 / stages[i]);
  }
  m_sprocketReductionRatio = ratio;
}

}  // namespace yams::gearing
