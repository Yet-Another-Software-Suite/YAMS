// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// SetupTelemetry(), Publish(), and ApplyTuningValues() are templates on the SwerveDrive's module
// count; their bodies live in SwerveDrive.hpp (see SwerveDriveTelemetry.hpp for why). This file
// only provides the non-templated constructor and Close().

#include "yams/telemetry/SwerveDriveTelemetry.hpp"

#include <utility>

namespace yams::telemetry {

SwerveDriveTelemetry::SwerveDriveTelemetry(SwerveDriveTelemetryConfig config)
    : m_config{std::move(config)} {}

void SwerveDriveTelemetry::Close() {
  for (auto& [field, dt] : m_config.GetDoubleFields()) dt.Close();
  for (auto& [field, stt] : m_config.GetPoseFields()) stt.Close();
  for (auto& [field, stt] : m_config.GetChassisSpeedsFields()) stt.Close();
  for (auto& [field, stat] : m_config.GetModuleStatesFields()) stat.Close();
}

}  // namespace yams::telemetry
