// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <wpi/commands2/CommandPtr.hpp>

#include "subsystems/ExampleSubsystem.h"

namespace autos {
/**
 * Example static factory for an autonomous command.
 */
wpi::cmd::CommandPtr ExampleAuto(ExampleSubsystem* subsystem);
}  // namespace autos
