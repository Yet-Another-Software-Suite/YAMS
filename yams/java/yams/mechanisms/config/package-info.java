// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Builder-style configuration classes for every mechanism in the YAMS library.
 *
 * <p>Every YAMS mechanism requires a corresponding config object to be constructed before the
 * mechanism itself is instantiated. Config classes follow a <b>fluent builder pattern</b>: each
 * {@code with*()} setter mutates the config and returns {@code this}, allowing calls to be
 * chained in a single expression.
 *
 * <h2>Available Configuration Classes</h2>
 * <ul>
 *   <li>{@link yams.mechanisms.config.ArmConfig} — single-jointed arm with angle limits and
 *       feedforward tuning</li>
 *   <li>{@link yams.mechanisms.config.ElevatorConfig} — linear elevator with height limits and
 *       gravity compensation</li>
 *   <li>{@link yams.mechanisms.config.PivotConfig} — pivot mechanism with continuous or bounded
 *       rotation</li>
 *   <li>{@link yams.mechanisms.config.FlyWheelConfig} — flywheel velocity mechanism with optional
 *       second follower motor</li>
 *   <li>{@link yams.mechanisms.config.SwerveDriveConfig} — full swerve-drive chassis geometry and
 *       module layout</li>
 *   <li>{@link yams.mechanisms.config.SwerveModuleConfig} — per-module drive and steer motor
 *       configuration</li>
 *   <li>{@link yams.mechanisms.config.DifferentialMechanismConfig} — differential (tank-drive)
 *       mechanism pairing two motors</li>
 *   <li>{@link yams.mechanisms.config.MechanismPositionConfig} — position targets and tolerance
 *       settings shared across positional mechanisms</li>
 *   <li>{@link yams.mechanisms.config.SensorConfig} — external encoder or absolute sensor
 *       attachment configuration</li>
 * </ul>
 *
 * <h2>Motor Controller Requirement</h2>
 * <p>A {@link yams.motorcontrollers.SmartMotorController} must be provided before most mechanisms
 * can be constructed. It may be supplied either through the config constructor or via the
 * {@code withSmartMotorController()} setter:
 *
 * <pre>{@code
 * SmartMotorController motor = new TalonFXController(new TalonFX(1));
 *
 * // Option A: pass in the constructor
 * ElevatorConfig config = new ElevatorConfig(motor)
 *     .withMaxHeight(Meters.of(1.5))
 *     .withGearing(9.0);
 *
 * // Option B: set via fluent API
 * ElevatorConfig config = new ElevatorConfig()
 *     .withSmartMotorController(motor)
 *     .withMaxHeight(Meters.of(1.5))
 *     .withGearing(9.0);
 * }</pre>
 *
 * <h2>Fluent Chaining Example</h2>
 * <pre>{@code
 * ArmConfig armConfig = new ArmConfig(motor)
 *     .withGearing(100.0)
 *     .withSoftLimits(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90))
 *     .withFeedforward(new ArmFeedforward(0.1, 0.5, 1.2))
 *     .withSensor(new SensorConfig(encoder).withOffset(Rotation2d.fromDegrees(15)));
 * }</pre>
 *
 * @see yams.mechanisms.SmartMechanism
 * @see yams.motorcontrollers.SmartMotorController
 */
package yams.mechanisms.config;
