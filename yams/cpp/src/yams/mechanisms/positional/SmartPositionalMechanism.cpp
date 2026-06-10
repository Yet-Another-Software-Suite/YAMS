// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/positional/SmartPositionalMechanism.hpp"

namespace yams::mechanisms::positional {

frc::MechanismLigament2d* SmartPositionalMechanism::GetMechanismLigament() {
  return m_mechanismLigament;
}

frc::MechanismRoot2d* SmartPositionalMechanism::GetMechanismRoot() { return m_mechanismRoot; }

motorcontrollers::SmartMotorController* SmartPositionalMechanism::GetMotor() {
  return GetMotorController();
}

}  // namespace yams::mechanisms::positional
