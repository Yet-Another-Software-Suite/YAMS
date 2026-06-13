// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Position-controlled mechanism implementations for the YAMS library.
 *
 * <p>This package contains concrete mechanism classes that track and control a <b>position
 * setpoint</b> (angle, height, or displacement) using closed-loop feedback. All classes extend
 * {@link yams.mechanisms.positional.SmartPositionalMechanism}, which provides the shared
 * closed-loop position logic, soft-limit enforcement, simulation wiring, and WPILib
 * command/trigger integration.
 *
 * <h2>Available Mechanisms</h2>
 * <ul>
 *   <li>{@link yams.mechanisms.positional.Arm} — single-jointed arm rotating about a fixed pivot,
 *       configured with {@link yams.mechanisms.config.ArmConfig}</li>
 *   <li>{@link yams.mechanisms.positional.Elevator} — linear-travel elevator stage, configured
 *       with {@link yams.mechanisms.config.ElevatorConfig}</li>
 *   <li>{@link yams.mechanisms.positional.Pivot} — generic pivot that may rotate continuously or
 *       within bounded limits, configured with {@link yams.mechanisms.config.PivotConfig}</li>
 *   <li>{@link yams.mechanisms.positional.DifferentialMechanism} — mechanism driven by two motors
 *       in a differential arrangement (e.g. differential wrist), configured with
 *       {@link yams.mechanisms.config.DifferentialMechanismConfig}</li>
 *   <li>{@link yams.mechanisms.positional.DoubleJointedArm} — two-segment arm whose second joint
 *       is kinematically dependent on the first, using two underlying
 *       {@link yams.mechanisms.positional.Arm} instances</li>
 * </ul>
 *
 * <h2>Setpoint Commands</h2>
 * <p>Setpoint methods return WPILib {@code Command} objects rather than setting the target
 * synchronously. This means they compose naturally with other commands and can be cancelled,
 * interrupted, or sequenced using the standard WPILib command scheduler:
 *
 * <pre>{@code
 * // Move the arm to 45 degrees and then print a message
 * arm.setAngle(Rotation2d.fromDegrees(45))
 *    .andThen(Commands.print("Arm at target"));
 *
 * // Move the elevator to 0.8 m, waiting until it arrives
 * elevator.setHeight(Meters.of(0.8));
 * }</pre>
 *
 * <h2>Trigger Factory Methods</h2>
 * <p>Each positional mechanism exposes factory methods that return
 * {@link edu.wpi.first.wpilibj2.command.button.Trigger} objects. These allow condition-based
 * command scheduling without manual polling in {@code periodic()}:
 * <ul>
 *   <li>{@code isNear(setpoint, tolerance)} — fires while the mechanism is within
 *       {@code tolerance} of {@code setpoint}</li>
 *   <li>{@code max()} — fires while the mechanism is at or beyond its configured maximum
 *       limit</li>
 *   <li>{@code min()} — fires while the mechanism is at or below its configured minimum
 *       limit</li>
 *   <li>{@code between(lower, upper)} — fires while the mechanism position is within the
 *       specified range</li>
 *   <li>{@code lte(position)} — fires while the mechanism position is less than or equal to
 *       {@code position}</li>
 *   <li>{@code gte(position)} — fires while the mechanism position is greater than or equal
 *       to {@code position}</li>
 * </ul>
 *
 * <pre>{@code
 * // Extend intake when arm clears 30 degrees
 * arm.gte(Rotation2d.fromDegrees(30))
 *    .onTrue(intakeSubsystem.extend());
 *
 * // Retract intake if arm drops below 30 degrees
 * arm.lte(Rotation2d.fromDegrees(30))
 *    .onTrue(intakeSubsystem.retract());
 * }</pre>
 *
 * @see yams.mechanisms.positional.SmartPositionalMechanism
 * @see yams.mechanisms.SmartMechanism
 * @see yams.mechanisms.config
 */
package yams.mechanisms.positional;
