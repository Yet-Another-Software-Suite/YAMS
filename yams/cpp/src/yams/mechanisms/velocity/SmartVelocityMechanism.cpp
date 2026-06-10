// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "yams/mechanisms/velocity/SmartVelocityMechanism.hpp"

namespace yams::mechanisms::velocity {

frc::MechanismLigament2d* SmartVelocityMechanism::GetMechanismLigament() {
  return m_mechanismLigament;
}

frc::MechanismRoot2d* SmartVelocityMechanism::GetMechanismRoot() { return m_mechanismRoot; }

motorcontrollers::SmartMotorController* SmartVelocityMechanism::GetMotor() {
  return GetMotorController();
}

}  // namespace yams::mechanisms::velocity
