// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import org.wpilib.backend.NetworkTablesTelemetryBackend;
import org.wpilib.backend.NetworkTablesTunableBackend;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.telemetry.TelemetryRegistry;
import org.wpilib.tunable.TunableRegistry;

/**
 * Registers {@link TelemetryRegistry} and {@link TunableRegistry} backends that publish directly
 * under the {@code Mechanisms} and {@code Tuning} NetworkTables roots so the same roots
 * {@link MechanismTelemetry} and {@link SmartMotorControllerTelemetry} already publish raw
 * NetworkTables data to — instead of the default {@code /Telemetry} and {@code /Tunables} roots.
 * This keeps dashboard-facing widgets ({@code Mechanism2d}, {@code Field2d}, on-dashboard
 * commands) in the same NetworkTables subtree as the rest of a mechanism's telemetry/tuning data.
 */
public final class NetworkTablesBackends {
  private static boolean mechanismsTelemetryRegistered = false;
  private static boolean mechanismsTunableRegistered   = false;
  private static boolean tuningTunableRegistered       = false;

  private NetworkTablesBackends() {
  }

  /**
   * Ensure a {@link TelemetryRegistry} backend is registered at {@code /Mechanisms} that publishes
   * directly to that NetworkTables root, for {@code TelemetryLoggable} objects such as
   * {@code Mechanism2d}.
   */
  public static synchronized void ensureMechanismsTelemetryBackend() {
    if (!mechanismsTelemetryRegistered) {
      TelemetryRegistry.registerBackend("/Mechanisms", new NetworkTablesTelemetryBackend(NetworkTableInstance.getDefault(), ""));
      mechanismsTelemetryRegistered = true;
    }
  }

  /**
   * Ensure a {@link TunableRegistry} backend is registered at {@code /Mechanisms} that publishes
   * directly to that NetworkTables root, for {@code ComplexTunable} objects such as
   * {@code Field2d} or on-dashboard {@code Command}s.
   */
  public static synchronized void ensureMechanismsTunableBackend() {
    if (!mechanismsTunableRegistered) {
      TunableRegistry.registerBackend("/Mechanisms", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), ""));
      mechanismsTunableRegistered = true;
    }
  }

  /**
   * Ensure a {@link TunableRegistry} backend is registered at {@code /Tuning} that publishes
   * directly to that NetworkTables root, for {@code ComplexTunable} on-dashboard {@code Command}s
   * such as the Live Tuning command.
   */
  public static synchronized void ensureTuningTunableBackend() {
    if (!tuningTunableRegistered) {
      TunableRegistry.registerBackend("/Tuning", new NetworkTablesTunableBackend(NetworkTableInstance.getDefault(), ""));
      tuningTunableRegistered = true;
    }
  }
}
