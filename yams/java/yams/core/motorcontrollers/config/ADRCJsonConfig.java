// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.config;

import io.avaje.jsonb.Json;

/**
 * On-disk representation of an {@link ADRCConfig}.
 *
 * <p>Every field is nullable so a file can omit anything it does not want to override. The loader
 * merges the non-null values onto the Java defaults in {@link ADRCConfig}; it never invents a
 * physical value such as {@code b0} or a voltage limit.
 *
 * <p>This type is package-private: teams edit the JSON, not this record.
 *
 * @param systemOrder    Plant order, 1 through 4.
 * @param settlingTime   Desired closed-loop settling time, seconds.
 * @param bandwidthScale Closed-loop bandwidth scale; larger is more aggressive.
 * @param b0             Approximate input gain, in mechanism units per volt per second squared.
 * @param minVoltage     Minimum controller output, volts.
 * @param maxVoltage     Maximum controller output, volts.
 * @param slewRateLimit  Maximum output change per second; zero disables the limit.
 * @param inputDelay     Input delay, seconds; zero disables the delay.
 * @param cascaded       Whether to run the cascaded ESO form.
 * @param tdMethod       Tracking differentiator method, or {@code "none"}.
 * @param tdParams       Tracking differentiator parameters.
 */
@Json
record ADRCJsonConfig(
    Integer systemOrder,
    Double settlingTime,
    Double bandwidthScale,
    Double b0,
    Double minVoltage,
    Double maxVoltage,
    Double slewRateLimit,
    Double inputDelay,
    Boolean cascaded,
    String tdMethod,
    double[] tdParams) {}
