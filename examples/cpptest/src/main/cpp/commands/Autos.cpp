// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#include "commands/Autos.h"

#include <wpi/commands2/Commands.hpp>

#include "commands/ExampleCommand.h"

wpi::cmd::CommandPtr autos::ExampleAuto(ExampleSubsystem* subsystem) {
  return wpi::cmd::Sequence(subsystem->ExampleMethodCommand(), ExampleCommand(subsystem).ToPtr());
}
