// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Physics-based simulation suppliers for YAMS motor controllers.
 *
 * <p>Simulation suppliers bridge WPILib's simulation framework and YAMS motor controller wrappers.
 * Pass an instance to {@link yams.motorcontrollers.SmartMotorControllerConfig} via
 * {@code withSimSupplier()} to enable physics-accurate simulation without changing mechanism code.
 *
 * <ul>
 *   <li>{@link yams.motorcontrollers.simulation.ArmSimSupplier} — single-jointed arm physics</li>
 *   <li>{@link yams.motorcontrollers.simulation.DCMotorSimSupplier} — generic DC motor physics</li>
 *   <li>{@link yams.motorcontrollers.simulation.Sensor} and {@link yams.motorcontrollers.simulation.SensorData} — simulated encoder/sensor state</li>
 * </ul>
 */
package yams.motorcontrollers.simulation;
