// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/gearing/MechanismGearing.hpp"

namespace yams::gearing {

const MechanismGearing MechanismGearing::kOne{1.0};

MechanismGearing::MechanismGearing(double reductionRatio)
    : m_gearBox(GearBox::FromReductionStages({reductionRatio})) {}

MechanismGearing::MechanismGearing(const GearBox& gearBox) : m_gearBox(gearBox) {}

MechanismGearing::MechanismGearing(const GearBox& gearBox, const Sprocket& sprockets)
    : m_gearBox(gearBox), m_sprockets(sprockets) {}

double MechanismGearing::GetRotorToMechanismRatio() const {
  double ratio = m_gearBox.GetInputToOutputConversionFactor();
  if (m_sprockets.has_value()) {
    ratio *= m_sprockets->GetInputToOutputConversionFactor();
  }
  return ratio;
}

double MechanismGearing::GetMechanismToRotorRatio() const {
  double ratio = m_gearBox.GetOutputToInputConversionFactor();
  if (m_sprockets.has_value()) {
    ratio *= m_sprockets->GetOutputToInputConversionFactor();
  }
  return ratio;
}

MechanismGearing& MechanismGearing::Div(double i) {
  m_gearBox.Div(i);
  return *this;
}

}  // namespace yams::gearing
