// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Motor-controller configuration objects that are not specific to a mechanism type.
 *
 * <p>{@link yams.core.motorcontrollers.config.ADRCConfig} is the ADRC counterpart of a PID
 * controller configuration. It is passed to
 * {@link yams.core.motorcontrollers.SmartMotorControllerConfig#withADRCController(ADRCConfig)} and
 * the motor controller runs the ADRC on its closed-loop thread, so mechanisms need no extra code.
 *
 * <p>Team-editable tuning lives in {@code src/main/deploy/adrc/<name>.json}, one file per
 * controller. The Java object carries the defaults, validation, and physical derivation; the JSON
 * carries the numbers a team tunes.
 *
 * <p>ADRC was <b>written by FRC Team 8331</b>.
 *
 * @see yams.core.motorcontrollers.config.ADRCConfig
 * @see yams.core.math.ADRCController
 */
package yams.core.motorcontrollers.config;
