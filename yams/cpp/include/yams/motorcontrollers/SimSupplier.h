// Copyright (c) 2026 YAMS Contributors
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

namespace yams::motorcontrollers {

class SimSupplier {
 public:
  virtual ~SimSupplier() = default;

  virtual void UpdateSim() = 0;

  virtual units::degree_t GetMechanismPosition() = 0;
  virtual units::degrees_per_second_t GetMechanismVelocity() = 0;
  virtual units::degree_t GetRotorPosition() = 0;
  virtual units::degrees_per_second_t GetRotorVelocity() = 0;

  virtual void SetMechanismPosition(units::degree_t angle) = 0;
  virtual void SetMechanismVelocity(units::degrees_per_second_t velocity) = 0;
  virtual void SetRotorPosition(units::degree_t angle) = 0;
  virtual void SetRotorVelocity(units::degrees_per_second_t velocity) = 0;

  virtual bool IsWatchdogExpired() = 0;
  virtual void FeedWatchdog() = 0;
};

}  // namespace yams::motorcontrollers
