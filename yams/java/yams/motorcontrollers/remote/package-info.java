// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * CTRE motor controller wrappers whose closed-loop control runs on the motor controller itself.
 *
 * <p>This package contains adapters for CTRE TalonFX-family motor controllers. Because the
 * Phoenix 6 firmware executes PID loops directly on the motor controller hardware, these
 * wrappers are classified as <em>remote</em>: setpoints are sent over CAN and the device
 * closes the loop autonomously at a high update rate without consuming roboRIO CPU cycles.
 *
 * <h2>Core types</h2>
 * <ul>
 *   <li>{@link yams.motorcontrollers.remote.TalonFXWrapper} — adapter for the CTRE TalonFX
 *       motor controller, used in the Kraken X60 and Falcon 500 brushless motors. Supports
 *       all Phoenix 6 control modes (position, velocity, motion magic, torque current)
 *       through the {@link yams.motorcontrollers.SmartMotorController} interface.</li>
 *   <li>{@link yams.motorcontrollers.remote.TalonFXSWrapper} — adapter for the CTRE TalonFXS
 *       motor controller, used with the Minion brushless motor. Functionally equivalent to
 *       {@code TalonFXWrapper} but targets the TalonFXS CAN device type.</li>
 * </ul>
 *
 * <h2>Phoenix 6 API</h2>
 * <p>Both wrappers are built on the <a href="https://v6.docs.ctr-electronics.com/">Phoenix 6
 * API</a>. Configuration is applied via YAMS' own
 * {@link yams.motorcontrollers.SmartMotorControllerConfig} fluent builder, which translates
 * YAMS settings into the appropriate {@code TalonFXConfiguration} or
 * {@code TalonFXSConfiguration} objects before pushing them to the device.
 *
 * <h2>CANcoder discontinuity</h2>
 * <p>When a CANcoder is fused as the remote sensor, specifying a discontinuity point via
 * {@link yams.motorcontrollers.SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}
 * is <strong>optional</strong>. If omitted, the CTRE default sensor range ({@code [0, 1)})
 * is used. Unlike the REV absolute encoder, the Phoenix 6 firmware handles the wrap
 * transparently in most configurations.
 *
 * <h2>Construction</h2>
 * <pre>{@code
 * // TalonFX (Kraken X60) on CAN ID 2
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withClosedLoopController(10.0,0,0.1).withStatorCurrentLimit(Amps.of(80));
 * SmartMotorController motor = new TalonFXWrapper(
 *     new TalonFX(2), DCMotor.getKrakenX60(1), config);
 *
 * // TalonFXS (Minion) on CAN ID 7
 * SmartMotorController miniMotor = new TalonFXSWrapper(
 *     new TalonFXS(7), DCMotor.getMinion(1), config);
 * }</pre>
 *
 * @see yams.motorcontrollers.SmartMotorControllerConfig
 */
package yams.motorcontrollers.remote;
