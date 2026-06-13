// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * Yet Another Mechanism System gearing classes for easier calculations.
 *
 * <p>This package provides gear-ratio math helpers that let teams express mechanism gearing in
 * terms of physical hardware (tooth counts, sprocket sizes, output dimensions) instead of
 * manually computing reduction fractions. All classes are immutable value types designed to be
 * constructed once at robot initialization and reused throughout the match.
 *
 * <h2>Core Classes</h2>
 * <ul>
 *   <li><b>{@link yams.gearing.GearBox}</b> — chains one or more gear stages together and
 *       computes the overall reduction ratio. Each stage is expressed as a pair of tooth counts
 *       (driver : driven). Multiple stages are multiplied automatically.</li>
 *   <li><b>{@link yams.gearing.Sprocket}</b> — models a chain or belt drive between two
 *       sprockets, computing the linear or rotational ratio from sprocket tooth counts and, for
 *       linear drives, the pitch diameter.</li>
 *   <li><b>{@link yams.gearing.MechanismGearing}</b> — ties a {@link yams.gearing.GearBox} to
 *       the physical output dimensions of a mechanism (e.g., spool radius for an elevator,
 *       arm length for a rotary mechanism) so that motor-side units can be converted directly to
 *       real-world position and velocity.</li>
 * </ul>
 *
 * <h2>Example — Single-Stage 5:1 Reduction</h2>
 * <p>Pass the driver tooth count first, then the driven tooth count. The ratio is
 * {@code driven / driver}, so {@code 12} driver teeth and {@code 60} driven teeth produces a
 * 5:1 reduction:
 * <pre>{@code
 * // Represents a single 12T-to-60T gear stage (5:1 overall reduction).
 * GearBox gearbox = new GearBox(12.0, 60.0);
 * double ratio = gearbox.getRatio(); // 5.0
 * }</pre>
 *
 * <h2>Example — Multi-Stage Gearbox</h2>
 * <pre>{@code
 * // Two stages: 12:60 then 14:50 → overall ratio = 5.0 × 3.571... ≈ 17.857:1
 * GearBox gearbox = new GearBox(12.0, 60.0, 14.0, 50.0);
 * }</pre>
 *
 * @see yams.gearing.GearBox
 * @see yams.gearing.Sprocket
 * @see yams.gearing.MechanismGearing
 * @see yams.exceptions.NoStagesGivenException
 * @see yams.exceptions.InvalidStageGivenException
 */
package yams.gearing;
