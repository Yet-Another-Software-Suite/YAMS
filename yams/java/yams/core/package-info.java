// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Yet Another Mechanism System (YAMS) the root package of the YAMS WPILib 2026 Java library.
 *
 * <p>YAMS abstracts common FRC robot mechanisms (arms, elevators, flywheels, swerve drives, and
 * more) behind a consistent configuration-driven API. Rather than hand-rolling PID loops,
 * gear-ratio math, and telemetry for every mechanism on the robot, teams describe their hardware
 * once in a config object and let YAMS handle the rest.
 *
 * <h2>Entry Points</h2>
 *
 * <p>There is no single top-level convenience class. The recommended starting point is one of the
 * mechanism config classes found in the {@code yams.core.mechanisms.config} sub-package, such as {@link
 * yams.core.mechanisms.config.ArmConfig}. Construct a config, supply the required hardware
 * parameters, then pass it to the corresponding mechanism class to obtain a fully configured,
 * telemetry-enabled mechanism instance.
 *
 * <h2>Package Overview</h2>
 *
 * <ul>
 * <li><b>{@code yams.core.mechanisms}</b> concrete mechanism implementations (arm, elevator,
 * flywheel, pivot, swerve, differential drive, double-jointed arm) and their config classes.
 * <li><b>{@code yams.core.motorcontrollers}</b> vendor-agnostic smart motor-controller wrappers that
 * normalize REV, CTRE, and other controllers behind a common interface.
 * <li><b>{@code yams.core.gearing}</b> gear-ratio helpers ({@link yams.core.gearing.GearBox}, {@link
 * yams.core.gearing.Sprocket}, {@link yams.core.gearing.MechanismGearing}) for expressing mechanism
 * gearing without manual ratio arithmetic.
 * <li><b>{@code yams.core.telemetry}</b> automatic publishing of mechanism state to AdvantageKit,
 * SmartDashboard, and other telemetry back-ends.
 * <li><b>{@code yams.core.math}</b> control-theory utilities including LQR, profiled PID, and
 * filtered derivative estimation.
 * <li><b>{@code yams.core.exceptions}</b> checked and unchecked exceptions thrown when mechanisms or
 * motor controllers are misconfigured.
 * </ul>
 *
 * <h2>Typical Usage</h2>
 *
 * <pre>{@code
 * // 1. Build a config describing the physical hardware.
 * ArmConfig config = new ArmConfig()
 *     .withMotor(new TalonFXController(1))
 *     .withGearing(new GearBox(12.0, 60.0))
 *     .withSoftLimits(Rotation2d.fromDegrees(-90), Rotation2d.fromDegrees(90));
 *
 * // 2. Construct the mechanism exceptions are thrown here if config is invalid.
 * SmartArm arm = new SmartArm(config);
 *
 * // 3. Command the mechanism from periodic or command-based code.
 * arm.setGoal(Rotation2d.fromDegrees(45));
 * arm.periodic();
 * }</pre>
 *
 * @see yams.core.mechanisms.config.ArmConfig
 * @see yams.core.gearing.GearBox
 * @see yams.core.exceptions
 */
package yams.core;
