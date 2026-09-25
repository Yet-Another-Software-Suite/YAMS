// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Commands v3 configuration classes for the YAMS library.
 *
 * <p>These classes extend the core configurations with the {@link org.wpilib.command3.Mechanism}
 * that the resulting commands require:
 *
 * <ul>
 * <li>{@link yams.commands3.config.SmartMotorControllerConfig} motor controller configuration,
 * also used to set up live tuning commands</li>
 * <li>{@link yams.commands3.config.SwerveDriveConfig} swerve drive configuration for
 * {@link yams.commands3.swerve.SwerveDrive}</li>
 * </ul>
 *
 * <p>Mechanism configurations (e.g. {@link yams.core.mechanisms.config.ArmConfig},
 * {@link yams.core.mechanisms.config.FlyWheelConfig}) have no command dependency and come from
 * {@link yams.core.mechanisms.config}.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * public class ShooterMechanism implements Mechanism {
 *   private final SmartMotorControllerConfig config = (SmartMotorControllerConfig)
 *       new SmartMotorControllerConfig(this)
 *           .withControlMode(ControlMode.CLOSED_LOOP)
 *           .withClosedLoopController(0.01, 0, 0)
 *           .withStatorCurrentLimit(Amps.of(40));
 * }
 * }</pre>
 *
 * @see yams.core.mechanisms.config
 * @see yams.core.motorcontrollers.SmartMotorController
 */
package yams.commands3.config;
