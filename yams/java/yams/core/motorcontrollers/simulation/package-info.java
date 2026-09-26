// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Physics-based simulation suppliers for YAMS motor controllers.
 *
 * <p>Simulation suppliers bridge WPILib's simulation framework and YAMS motor controller wrappers.
 * Pass an instance to {@link yams.core.motorcontrollers.SmartMotorControllerConfig} via
 * {@code withSimSupplier()} to enable physics-accurate simulation without changing mechanism code.
 *
 * <ul>
 * <li>{@link yams.core.motorcontrollers.simulation.ArmSimSupplier} single-jointed arm physics
 * <li>{@link yams.core.motorcontrollers.simulation.DCMotorSimSupplier} generic DC motor physics
 * <li>{@link yams.core.motorcontrollers.simulation.Sensor} and
 * {@link yams.core.motorcontrollers.simulation.SensorData} simulated encoder/sensor state
 * </ul>
 */
package yams.core.motorcontrollers.simulation;
