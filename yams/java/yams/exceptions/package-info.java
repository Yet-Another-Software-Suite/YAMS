// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Exceptions for YAMS. Will tell you what went wrong and how to fix it.
 *
 * <p>This package contains the checked and unchecked exceptions thrown by YAMS when mechanisms or
 * motor controllers are misconfigured. All exceptions include a human-readable message that
 * identifies the missing or invalid field and, where possible, suggests the corrective action.
 * They are typically thrown during mechanism construction — before any control loop begins — so
 * configuration errors surface immediately at robot startup rather than silently producing
 * unexpected behavior at match time.
 *
 * <h2>Per-Mechanism Configuration Exceptions</h2>
 * <p>Each mechanism type has a dedicated exception thrown when its required configuration fields
 * are absent or logically inconsistent:
 * <ul>
 *   <li>{@link yams.exceptions.ArmConfigurationException} — single-jointed arm config errors</li>
 *   <li>{@link yams.exceptions.ElevatorConfigurationException} — elevator config errors</li>
 *   <li>{@link yams.exceptions.FlyWheelConfigurationException} — flywheel config errors</li>
 *   <li>{@link yams.exceptions.PivotConfigurationException} — pivot config errors</li>
 *   <li>{@link yams.exceptions.SwerveDriveConfigurationException} — swerve drive config errors</li>
 *   <li>{@link yams.exceptions.DifferentialMechanismConfigurationException} — differential
 *       drive config errors</li>
 *   <li>{@link yams.exceptions.DoubleJointedArmConfigurationException} — double-jointed arm
 *       config errors</li>
 * </ul>
 *
 * <h2>Motor Controller Exceptions</h2>
 * <p>Thrown when a motor controller is missing or its configuration is invalid:
 * <ul>
 *   <li>{@link yams.exceptions.SmartMotorControllerConfigurationException} — the motor
 *       controller wrapper itself has an invalid or incomplete configuration</li>
 *   <li>{@link yams.exceptions.MotorNotPresentException} — a required motor controller could
 *       not be found on the CAN bus or is otherwise unavailable</li>
 * </ul>
 *
 * <h2>Gear Stage Exceptions</h2>
 * <p>Thrown by {@link yams.gearing.GearBox} when its list of stages is malformed:
 * <ul>
 *   <li>{@link yams.exceptions.NoStagesGivenException} — a {@code GearBox} was constructed
 *       with an empty stage list</li>
 *   <li>{@link yams.exceptions.InvalidStageGivenException} — one or more stages in a
 *       {@code GearBox} have an illegal value (e.g., zero or negative tooth counts)</li>
 * </ul>
 */
package yams.exceptions;
