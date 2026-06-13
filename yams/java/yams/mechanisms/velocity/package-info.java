// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Velocity-controlled mechanism implementations for the YAMS library.
 *
 * <p>This package contains concrete mechanism classes that regulate a <b>velocity setpoint</b>
 * rather than a position. All classes extend
 * {@link yams.mechanisms.velocity.SmartVelocityMechanism}, which provides closed-loop velocity
 * control, feedforward integration, simulation wiring, and WPILib command/trigger support.
 *
 * <h2>Available Mechanisms</h2>
 * <ul>
 *   <li>{@link yams.mechanisms.velocity.FlyWheel} — single or dual-motor flywheel configured with
 *       {@link yams.mechanisms.config.FlyWheelConfig}. Accepts both
 *       {@link edu.wpi.first.units.measure.AngularVelocity} (e.g. RPM) and
 *       {@link edu.wpi.first.units.measure.LinearVelocity} (e.g. surface speed in m/s) setpoints
 *       when a wheel radius is provided in the config.</li>
 * </ul>
 *
 * <h2>Running the Mechanism</h2>
 * <ul>
 *   <li>{@code run(velocity)} — commands the mechanism to spin at the given velocity. Accepts
 *       either {@link edu.wpi.first.units.measure.AngularVelocity} or
 *       {@link edu.wpi.first.units.measure.LinearVelocity} and returns a {@code Command} that
 *       holds the setpoint until interrupted.</li>
 *   <li>{@code runTo(velocity)} — like {@code run()}, but the returned {@code Command} does not
 *       finish until the mechanism is within the configured velocity tolerance of the setpoint.
 *       Useful when downstream actions must wait for the flywheel to spin up.</li>
 * </ul>
 *
 * <pre>{@code
 * FlyWheelConfig config = new FlyWheelConfig(motor)
 *     .withWheelRadius(Inches.of(2))
 *     .withVelocityTolerance(RotationsPerSecond.of(1));
 *
 * FlyWheel flyWheel = new FlyWheel(config);
 *
 * // Spin at a linear surface speed and wait until up to speed before shooting
 * flyWheel.runTo(MetersPerSecond.of(10))
 *         .andThen(shooter.fireCommand())
 *         ;
 *
 * // Spin at an angular velocity (does not wait for spin-up)
 * flyWheel.run(RotationsPerSecond.of(50));
 * }</pre>
 *
 * <h2>Unsupported Operations</h2>
 * <p>Because velocity mechanisms do not track an absolute position, the positional limit triggers
 * {@code max()} and {@code min()} are <b>not supported</b> and will throw
 * {@link java.lang.UnsupportedOperationException} if called. Use the velocity-based triggers
 * ({@code isNear()}, {@code gte()}, {@code lte()}) for condition-based scheduling instead.
 *
 * @see yams.mechanisms.velocity.SmartVelocityMechanism
 * @see yams.mechanisms.SmartMechanism
 * @see yams.mechanisms.config.FlyWheelConfig
 */
package yams.mechanisms.velocity;
