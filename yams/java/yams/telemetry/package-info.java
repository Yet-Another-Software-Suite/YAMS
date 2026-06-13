// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * NetworkTables publishers for mechanism and motor controller state.
 *
 * <p>This package provides classes that stream live data from YAMS mechanisms and motor
 * controllers to a NetworkTables server, making that data available to dashboard tools
 * (Shuffleboard, AdvantageScope, Glass) and to tuning workflows that adjust PID gains
 * without redeploying code.
 *
 * <h2>Mechanism telemetry</h2>
 * <ul>
 *   <li>{@link yams.telemetry.MechanismTelemetry} — publishes the high-level state of a
 *       mechanism: current position, active setpoint, goal, and status flags. Updated each
 *       robot periodic cycle.</li>
 * </ul>
 *
 * <h2>Motor controller telemetry</h2>
 * <ul>
 *   <li>{@link yams.telemetry.SmartMotorControllerTelemetryConfig} — fluent builder that
 *       selects which signals to publish and under what NetworkTables keys. Signals include
 *       duty cycle, applied voltage, velocity, position, supply current, stator current, and
 *       device temperature.</li>
 *   <li>{@link yams.telemetry.SmartMotorControllerTelemetry} — publisher that consumes a
 *       {@code SmartMotorControllerTelemetryConfig} and writes the selected signals to
 *       NetworkTables on every {@code periodic()} call.</li>
 * </ul>
 *
 * <h2>Lightweight scalar publishers</h2>
 * <ul>
 *   <li>{@link yams.telemetry.BooleanTelemetry} — lightweight publisher for a single
 *       {@code boolean} value (e.g. limit switch state, at-setpoint flag).</li>
 *   <li>{@link yams.telemetry.DoubleTelemetry} — lightweight publisher for a single
 *       {@code double} value (e.g. a raw sensor reading or computed error).</li>
 * </ul>
 *
 * <h2>Enabling telemetry</h2>
 * <p>Telemetry is opt-in. Call {@code setupTelemetry()} on a
 * {@link yams.motorcontrollers.SmartMotorController} or mechanism instance, passing the
 * desired configuration object, to begin publishing. The call must happen before the first
 * {@code periodic()} call (typically in {@code robotInit} or the subsystem constructor):
 * <pre>{@code
 * SmartMotorControllerTelemetryConfig telemetryCfg =
 *     new SmartMotorControllerTelemetryConfig("arm/motor")
 *         .withVelocity()
 *         .withPosition()
 *         .withSupplyCurrent()
 *         .withTemperature();
 *
 * motor.setupTelemetry(telemetryCfg);
 * }</pre>
 *
 * @see yams.motorcontrollers.SmartMotorController
 */
package yams.telemetry;
