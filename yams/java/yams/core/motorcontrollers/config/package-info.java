// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Motor-controller configuration objects that are not specific to a mechanism type.
 *
 * <p>{@link yams.core.motorcontrollers.config.ADRCConfig} is the active disturbance rejection
 * counterpart of a PID controller configuration. It is passed to
 * {@link yams.core.motorcontrollers.SmartMotorControllerConfig#withADRCController(ADRCConfig)} and
 * the motor controller runs the controller on its closed-loop thread, so mechanisms need no extra
 * code.
 *
 * <p>Like the other YAMS config classes, {@link yams.core.motorcontrollers.config.ADRCConfig} is a
 * plain builder: teams set the values they tune with its {@code withX(...)} methods, usually from
 * their {@code Constants} class, while the object itself carries the defaults, validation, and
 * physical derivation.
 *
 * <p>ADRC was <b>written by FRC Team 8331</b>.
 *
 * @see yams.core.motorcontrollers.config.ADRCConfig
 * @see yams.core.math.ADRCController
 */
package yams.core.motorcontrollers.config;
