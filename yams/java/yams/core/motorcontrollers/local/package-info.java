// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * REV Robotics motor controller wrappers whose closed-loop control runs on the roboRIO.
 *
 * <p>This package contains adapters for REV SPARK MAX and SPARK FLEX motor controllers. Because the
 * REV hardware offloads PID execution to the roboRIO (rather than to the motor controller itself),
 * these wrappers are classified as <em>local</em>: the control loop runs on the robot controller
 * and sends periodic duty-cycle or voltage commands to the hardware.
 *
 * <h2>Core type</h2>
 *
 * <ul>
 * <li>{@link yams.core.motorcontrollers.local.SparkWrapper} unified adapter for both the SPARK
 * MAX (NEO, NEO 550) and SPARK FLEX (NEO Vortex).
 * </ul>
 *
 * <h2>Absolute encoder range</h2>
 *
 * <p>An absolute encoder, such as a REV Through Bore Encoder on a {@code SparkAbsoluteEncoder},
 * reads [0, 1) rotations. With
 * {@link yams.core.motorcontrollers.SmartMotorControllerConfig#withContinuousWrapping}, it reads the
 * wrapping range instead, with the encoder on the mechanism:
 *
 * <ul>
 * <li>{@code [-0.5, 0.5)}: when the mechanism travels through zero.
 * <li>{@code [0, 1)}: when zero is never crossed.
 * </ul>
 *
 * <h2>Construction</h2>
 *
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withClosedLoopController(4, 0, 0)
 *     .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5));
 * SmartMotorController motor = new SparkWrapper(
 *     new SparkMax(5, MotorType.kBrushless), DCMotor.getNEO(1), config);
 * }</pre>
 *
 * @see yams.core.motorcontrollers.SmartMotorControllerConfig
 */
package yams.core.motorcontrollers.local;
