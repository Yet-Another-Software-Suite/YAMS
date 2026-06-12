// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

// Mirrors Java SchedulerPumpHelper + TestWithScheduler.
// Pumps the WPILib CommandScheduler in fixed 20 ms steps for a given
// duration, advancing HAL simulation time each step.

#include <frc/simulation/SimHooks.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <units/time.h>

#include <chrono>
#include <functional>
#include <thread>

namespace yams::test {

class SchedulerHelper {
 public:
  static void Schedule(frc2::Command* cmd) { frc2::CommandScheduler::GetInstance().Schedule(cmd); }

  // Run the scheduler for `duration`, calling `cycleCallback` at the end of
  // each 20 ms step.  Pass nullptr for cycleCallback to omit the callback.
  static void RunForDuration(units::second_t duration,
                             std::function<void()> cycleCallback = nullptr, int heartbeatMs = 20) {
    int steps = static_cast<int>(units::millisecond_t{duration}.value() / heartbeatMs);
    for (int i = 0; i < steps; ++i) {
      frc2::CommandScheduler::GetInstance().Run();
      frc::sim::StepTiming(units::millisecond_t{static_cast<double>(heartbeatMs)});
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      if (cycleCallback) cycleCallback();
    }
  }

  static void Enable() { frc2::CommandScheduler::GetInstance().Enable(); }

  static void CancelAll() { frc2::CommandScheduler::GetInstance().CancelAll(); }
};

}  // namespace yams::test
