// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Hardware-agnostic motor controller abstractions for FRC mechanisms.
 *
 * <p>This package provides a unified API for controlling brushless motor controllers from multiple
 * vendors. All concrete wrappers extend {@link yams.motorcontrollers.SmartMotorController}, which
 * exposes position, velocity, and voltage control modes without reference to any vendor-specific
 * types.
 *
 * <h2>Core types</h2>
 * <ul>
 *   <li>{@link yams.motorcontrollers.SmartMotorController} — the hardware-agnostic abstract base
 *       that all vendor wrappers extend. Defines the lifecycle methods ({@code configure},
 *       {@code periodic}, {@code stop}) and the control interface (position, velocity, voltage,
 *       duty-cycle setpoints).</li>
 *   <li>{@link yams.motorcontrollers.SmartMotorControllerConfig} — a fluent builder used to
 *       configure every aspect of a motor before it is applied. Settings include PID gains,
 *       feedforward constants, position and velocity software limits, ramp rates, current limits,
 *       encoder resolution and offsets, and gear ratios. Call
 *       {@code SmartMotorController.configure(config)} to push all settings to hardware.</li>
 *   <li>{@link yams.motorcontrollers.SmartMotorControllerCommandRegistry} — registers
 *       WPILib {@code Command} objects that operate a specific motor so they can be
 *       discovered and scheduled by a mechanism.</li>
 * </ul>
 *
 * <h2>Usage example</h2>
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withPID(0.1, 0.0, 0.001)
 *     .withCurrentLimit(40)
 *     .withPositionConversionFactor(2 * Math.PI / 42.0);
 *
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(1, MotorType.kBrushless), DCMotor.getNEO(1), config);
 * motor.setPositionSetpoint(Math.PI / 2);
 * }</pre>
 *
 * <h2>Sub-packages</h2>
 * <ul>
 *   <li>{@link yams.motorcontrollers.local} — REV Robotics SPARK MAX and SPARK FLEX wrappers
 *       that run closed-loop control on the roboRIO.</li>
 *   <li>{@link yams.motorcontrollers.remote} — CTRE TalonFX and TalonFXS wrappers that run
 *       closed-loop control on the motor controller itself via the Phoenix 6 API.</li>
 *   <li>{@code yams.motorcontrollers.simulation} — simulation-only implementations used in
 *       unit tests and the WPILib simulation GUI.</li>
 * </ul>
 */
package yams.motorcontrollers;
