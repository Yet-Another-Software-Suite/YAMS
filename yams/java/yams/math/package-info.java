// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Yet Another Mechanism System math classes for easier calculations.
 *
 * <p>This package contains control-theory math utilities used internally by YAMS mechanisms and
 * available for direct use by teams that need low-level control primitives. All classes are
 * designed to integrate seamlessly with WPILib 2026 unit types and the rest of the YAMS API.
 *
 * <h2>Class Overview</h2>
 * <ul>
 *   <li><b>{@link yams.math.LQRController}</b> and <b>{@link yams.math.LQRConfig}</b> —
 *       implement discrete Linear Quadratic Regulator (LQR) control. {@code LQRConfig} holds the
 *       Q (state cost) and R (input cost) weighting matrices along with the plant model;
 *       {@code LQRController} solves the discrete algebraic Riccati equation at construction time
 *       and exposes a {@code calculate()} method suitable for calling from a periodic loop.</li>
 *   <li><b>{@link yams.math.DerivativeTimeFilter}</b> — computes a filtered derivative estimate
 *       for use as the D term in PID controllers. Rather than a raw finite difference (which
 *       amplifies sensor noise), this class applies a configurable low-pass filter to the
 *       derivative signal, trading off noise rejection against phase lag.</li>
 *   <li><b>{@link yams.math.SmartMath}</b> — a collection of static helper methods for common
 *       control-theory and unit-conversion tasks (clamping, interpolation, wrapping angles,
 *       etc.) that do not belong to a specific controller class.</li>
 *   <li><b>{@link yams.math.ExponentialProfilePIDController}</b> — a profiled PID variant that
 *       uses an exponential motion profile to generate setpoints. Compared to a trapezoidal
 *       profile, the exponential profile produces smoother acceleration transitions and is less
 *       prone to overshoot on mechanisms with significant inertia.</li>
 * </ul>
 *
 * <h2>Example — LQR Setup</h2>
 * <pre>{@code
 * // Build a config for a single-state, single-input system (e.g., flywheel velocity).
 * LQRConfig config = new LQRConfig()
 *     .withQWeights(1.0)   // state cost: penalise velocity error
 *     .withRWeights(12.0)  // input cost: penalise voltage
 *     .withPlant(plant);   // LinearSystem from SysId or known constants
 *
 * LQRController lqr = new LQRController(config);
 *
 * // In periodic():
 * double voltage = lqr.calculate(currentVelocity, goalVelocity);
 * motor.setVoltage(voltage);
 * }</pre>
 *
 * <h2>Example — Filtered Derivative</h2>
 * <pre>{@code
 * DerivativeTimeFilter dFilter = new DerivativeTimeFilter(0.02 /* dt seconds *\/);
 *
 * // In periodic():
 * double filteredDeriv = dFilter.calculate(measurement);
 * }</pre>
 *
 * @see yams.math.LQRController
 * @see yams.math.LQRConfig
 * @see yams.math.DerivativeTimeFilter
 * @see yams.math.SmartMath
 * @see yams.math.ExponentialProfilePIDController
 */
package yams.math;
