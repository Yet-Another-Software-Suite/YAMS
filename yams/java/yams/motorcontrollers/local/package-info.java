// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * REV Robotics motor controller wrappers whose closed-loop control runs on the roboRIO.
 *
 * <p>This package contains adapters for REV SPARK MAX and SPARK FLEX motor controllers.
 * Because the REV hardware offloads PID execution to the roboRIO (rather than to the motor
 * controller itself), these wrappers are classified as <em>local</em>: the control loop runs
 * on the robot controller and sends periodic duty-cycle or voltage commands to the hardware.
 *
 * <h2>Core type</h2>
 * <ul>
 *   <li>{@link yams.motorcontrollers.local.SparkWrapper} — unified adapter for both the
 *       SPARK MAX (NEO, NEO 550) and SPARK FLEX (NEO Vortex). Automatically detects the
 *       hardware variant from the {@code DCMotor} type passed to
 *       {@link yams.motorcontrollers.SmartMotorFactory}.</li>
 * </ul>
 *
 * <h2>Absolute encoder discontinuity</h2>
 * <p>When a {@code SparkAbsoluteEncoder} (REV Through Bore Encoder) is used as the feedback
 * device, a <em>discontinuity point</em> <strong>must</strong> be configured so the SPARK
 * firmware knows where the sensor wraps around. Set this via
 * {@link yams.motorcontrollers.SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}:
 * <ul>
 *   <li>{@code 0.5} — sensor range {@code [-0.5, 0.5)}: useful when the mechanism travels
 *       through zero and must not have a wrap at the midpoint of travel.</li>
 *   <li>{@code 1.0} — sensor range {@code [0, 1)}: useful when the mechanism only moves in
 *       the positive direction and zero is never crossed.</li>
 * </ul>
 * Omitting this setting when an absolute encoder is attached will produce a configuration
 * warning and may cause erratic position readings near the wrap boundary.
 *
 * <h2>Construction</h2>
 * <p>Do not instantiate {@link yams.motorcontrollers.local.SparkWrapper} directly. Use
 * {@link yams.motorcontrollers.SmartMotorFactory} instead so the correct hardware type is
 * selected automatically:
 * <pre>{@code
 * // Creates a SparkWrapper backed by a SPARK MAX on CAN ID 5
 * SmartMotorController motor = SmartMotorFactory.create(DCMotor.getNEO(1), 5);
 *
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withExternalEncoderDiscontinuityPoint(0.5);
 * motor.configure(config);
 * }</pre>
 *
 * @see yams.motorcontrollers.SmartMotorFactory
 * @see yams.motorcontrollers.SmartMotorControllerConfig
 */
package yams.motorcontrollers.local;
