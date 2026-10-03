// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

#pragma once

#include <memory>

#include "wpi/backend/NetworkTablesTelemetryBackend.hpp"
#include "wpi/backend/NetworkTablesTunableBackend.hpp"
#include "wpi/nt/NetworkTableInstance.hpp"
#include "wpi/telemetry/TelemetryRegistry.hpp"
#include "wpi/tunables/TunableRegistry.hpp"

namespace yams::telemetry {

/**
 * Registers TelemetryRegistry and TunableRegistry backends that publish directly under the
 * Mechanisms and Tuning NetworkTables roots -- the same roots MechanismTelemetry and
 * SmartMotorControllerTelemetry already publish raw NetworkTables data to -- instead of the
 * default /Telemetry and /Tunables roots. This keeps dashboard-facing widgets (Mechanism2d,
 * Field2d, on-dashboard commands) in the same NetworkTables subtree as the rest of a mechanism's
 * telemetry/tuning data.
 */

/**
 * Ensure a TelemetryRegistry backend is registered at /Mechanisms that publishes directly to
 * that NetworkTables root, for TelemetryLoggable objects such as Mechanism2d.
 */
inline void EnsureMechanismsTelemetryBackend() {
  static bool registered = [] {
    wpi::telemetry::TelemetryRegistry::RegisterBackend(
        "/Mechanisms", std::make_shared<wpi::backend::NetworkTablesTelemetryBackend>(
                           wpi::nt::NetworkTableInstance::GetDefault(), ""));
    return true;
  }();
  (void)registered;
}

/**
 * Ensure a TunableRegistry backend is registered at /Mechanisms that publishes directly to
 * that NetworkTables root, for ComplexTunable objects such as Field2d or on-dashboard Commands.
 */
inline void EnsureMechanismsTunableBackend() {
  static bool registered = [] {
    wpi::tunables::TunableRegistry::RegisterBackend(
        "/Mechanisms", std::make_shared<wpi::backend::NetworkTablesTunableBackend>(
                           wpi::nt::NetworkTableInstance::GetDefault(), ""));
    return true;
  }();
  (void)registered;
}

/**
 * Ensure a TunableRegistry backend is registered at /Tuning that publishes directly to that
 * NetworkTables root, for ComplexTunable on-dashboard Commands such as the Live Tuning command.
 */
inline void EnsureTuningTunableBackend() {
  static bool registered = [] {
    wpi::tunables::TunableRegistry::RegisterBackend(
        "/Tuning", std::make_shared<wpi::backend::NetworkTablesTunableBackend>(
                       wpi::nt::NetworkTableInstance::GetDefault(), ""));
    return true;
  }();
  (void)registered;
}

}  // namespace yams::telemetry
