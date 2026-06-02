// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <optional>

#include "GearBox.h"
#include "Sprocket.h"

namespace yams::gearing {

class MechanismGearing {
 public:
  static const MechanismGearing kOne;

  explicit MechanismGearing(double reductionRatio);
  explicit MechanismGearing(const GearBox& gearBox);
  MechanismGearing(const GearBox& gearBox, const Sprocket& sprockets);

  double GetRotorToMechanismRatio() const;
  double GetMechanismToRotorRatio() const;

  MechanismGearing& Div(double i);

 private:
  GearBox m_gearBox;
  std::optional<Sprocket> m_sprockets;
};

}  // namespace yams::gearing
