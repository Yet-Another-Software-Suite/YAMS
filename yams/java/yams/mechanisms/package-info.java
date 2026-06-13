// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Top-level mechanism abstractions for the YAMS (Yet Another Mechanism System) library.
 *
 * <p>This package contains {@link yams.mechanisms.SmartMechanism}, the abstract base class from
 * which every YAMS mechanism inherits. It defines the shared lifecycle, sensor handling, simulation
 * support, and WPILib command/trigger integration that all concrete mechanism types rely on.
 *
 * <h2>Mechanism Lifecycle</h2>
 * <ol>
 *   <li><b>Create a configuration object</b> — choose the appropriate config class from
 *       {@link yams.mechanisms.config} (e.g. {@link yams.mechanisms.config.ArmConfig},
 *       {@link yams.mechanisms.config.ElevatorConfig}) and populate it via the fluent
 *       {@code with*()} API.</li>
 *   <li><b>Construct the mechanism</b> — pass the finished config to the mechanism constructor
 *       (e.g. {@link yams.mechanisms.positional.Arm}, {@link yams.mechanisms.positional.Elevator}).
 *       The constructor validates the config and initialises all internal controllers.</li>
 *   <li><b>Command the mechanism</b> — call setpoint methods such as {@code setAngle()} or
 *       {@code setHeight()}. Each method returns a WPILib {@code Command} that can be bound,
 *       composed, or scheduled directly via the command scheduler.</li>
 *   <li><b>React to conditions</b> — use trigger factory methods ({@code isNear()}, {@code max()},
 *       {@code min()}, {@code between()}, {@code lte()}, {@code gte()}) to obtain
 *       {@link edu.wpi.first.wpilibj2.command.button.Trigger} objects that fire when the mechanism
 *       satisfies a particular condition, enabling condition-based command scheduling without
 *       polling in {@code periodic()}.</li>
 * </ol>
 *
 * <h2>Typical Usage</h2>
 * <pre>{@code
 * // 1. Configure
 * ArmConfig config = new ArmConfig(motor)
 *     .withGearing(100.0)
 *     .withSoftLimits(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90));
 *
 * // 2. Construct
 * Arm arm = new Arm(config);
 *
 * // 3. Schedule a setpoint command
 * arm.setAngle(Rotation2d.fromDegrees(45));
 *
 * // 4. Bind a trigger
 * arm.isNear(Rotation2d.fromDegrees(45), Rotation2d.fromDegrees(2))
 *    .onTrue(Commands.print("Arm at target"));
 * }</pre>
 *
 * <h2>Sub-packages</h2>
 * <ul>
 *   <li>{@link yams.mechanisms.positional} — position-controlled mechanisms (Arm, Elevator,
 *       Pivot, DifferentialMechanism, DoubleJointedArm)</li>
 *   <li>{@link yams.mechanisms.velocity} — velocity-controlled mechanisms (FlyWheel)</li>
 *   <li>{@link yams.mechanisms.swerve} — swerve-drive mechanism and module implementations</li>
 *   <li>{@link yams.mechanisms.config} — builder-style configuration classes for all mechanisms</li>
 * </ul>
 *
 * @see yams.mechanisms.SmartMechanism
 * @see yams.mechanisms.positional
 * @see yams.mechanisms.velocity
 * @see yams.mechanisms.swerve
 * @see yams.mechanisms.config
 */
package yams.mechanisms;
