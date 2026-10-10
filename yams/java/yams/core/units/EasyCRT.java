// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.units;

import static org.wpilib.units.Units.Rotations;

import java.util.Optional;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.Angle;

/**
 * Finds the absolute angle of a gear that turns more than once, from two absolute encoders on
 * pinions with different tooth counts that both mesh with it.
 *
 * <p>Each encoder only knows its angle within one of its own rotations, but the two readings
 * together are unique over {@link #getUniqueRange()}, which is
 * {@code lcm(encoderATeeth, encoderBTeeth) / sharedGearTeeth} rotations of the shared gear.
 *
 * <p>Both encoders must be zeroed with the mechanism at zero, and both must read increasing angles
 * when the mechanism moves in its positive direction. Set the offset and inversion on the encoders
 * themselves, or adjust the readings before passing them in.
 *
 * <p>This is not a textbook Chinese Remainder Theorem solve. Every angle consistent with encoder A
 * is tried, and the one whose predicted encoder B reading is closest to the real one wins, which
 * stays stable with backlash and sensor noise. Created by team 6911.
 *
 * <pre>{@code
 * // 100 tooth turret gear, 19 tooth pinion on encoder A, 21 tooth pinion on encoder B.
 * EasyCRT crt = new EasyCRT(100, 19, 21);
 *
 * // Read once at startup, while the mechanism is not moving.
 * Optional<Angle> turretAngle = crt.solve(encoderA.get(), encoderB.get());
 * }</pre>
 */
public class EasyCRT {
  /** Fraction of the gap between neighboring candidates that a match may be off by. */
  private static final double TOLERANCE_FRACTION = 0.25;

  /** Encoder A rotations per rotation of the shared gear. */
  private final double ratioA;

  /** Encoder B rotations per rotation of the shared gear. */
  private final double ratioB;

  /** Rotations of the shared gear over which the pair of encoder readings is unique. */
  private final double uniqueRangeRotations;

  /** Largest difference, in encoder B rotations, between its predicted and real reading. */
  private final double toleranceRotations;

  /**
   * Create a solver from the tooth counts of the three gears.
   *
   * @param sharedGearTeeth Teeth on the gear both encoder pinions mesh with. The solved angle is
   *                        the angle of this gear.
   * @param encoderATeeth   Teeth on encoder A's pinion.
   * @param encoderBTeeth   Teeth on encoder B's pinion.
   * @throws IllegalArgumentException if any tooth count is zero or negative.
   */
  public EasyCRT(int sharedGearTeeth, int encoderATeeth, int encoderBTeeth) {
    if (sharedGearTeeth <= 0 || encoderATeeth <= 0 || encoderBTeeth <= 0) {
      throw new IllegalArgumentException("Gear tooth counts must be positive");
    }
    int commonTeeth = gcd(encoderATeeth, encoderBTeeth);
    ratioA = (double) sharedGearTeeth / encoderATeeth;
    ratioB = (double) sharedGearTeeth / encoderBTeeth;
    // The readings repeat together every lcm(A, B) teeth of the shared gear.
    uniqueRangeRotations = (double) encoderATeeth / commonTeeth * encoderBTeeth / sharedGearTeeth;
    // Neighboring candidates predict encoder B readings this many rotations apart.
    toleranceRotations = TOLERANCE_FRACTION * commonTeeth / encoderBTeeth;
  }

  /**
   * Solve for the angle of the shared gear, assuming its travel is centered on zero. The result is
   * within half of {@link #getUniqueRange()} either side of zero.
   *
   * @param encoderA Encoder A reading.
   * @param encoderB Encoder B reading.
   * @return The shared gear's angle, or empty if the two readings do not agree on one.
   */
  public Optional<Angle> solve(Angle encoderA, Angle encoderB) {
    return solve(encoderA, encoderB, -uniqueRangeRotations / 2.0, uniqueRangeRotations / 2.0);
  }

  /**
   * Solve for the angle of the shared gear when its travel is not centered on zero.
   *
   * @param encoderA Encoder A reading.
   * @param encoderB Encoder B reading.
   * @param minAngle Lowest angle the shared gear can be at.
   * @param maxAngle Highest angle the shared gear can be at.
   * @return The shared gear's angle, or empty if the two readings do not agree on one.
   * @throws IllegalArgumentException if the travel is longer than {@link #getUniqueRange()}, or
   *                                  {@code maxAngle} is below {@code minAngle}.
   */
  public Optional<Angle> solve(Angle encoderA, Angle encoderB, Angle minAngle, Angle maxAngle) {
    double minRotations = minAngle.in(Rotations);
    double maxRotations = maxAngle.in(Rotations);
    if (maxRotations < minRotations) {
      throw new IllegalArgumentException("maxAngle must not be below minAngle");
    }
    if (maxRotations - minRotations > uniqueRangeRotations + 1e-9) {
      throw new IllegalArgumentException("Travel of " + (maxRotations - minRotations) + " rotations is longer than the " + uniqueRangeRotations + " rotations these gears can tell apart");
    }
    return solve(encoderA, encoderB, minRotations, maxRotations);
  }

  /**
   * Get how far the shared gear can turn before the pair of encoder readings repeats.
   *
   * @return The unique range of the shared gear.
   */
  public Angle getUniqueRange() {
    return Rotations.of(uniqueRangeRotations);
  }

  /**
   * Try every shared gear angle consistent with encoder A and keep the one that best predicts
   * encoder B.
   *
   * @param encoderA     Encoder A reading.
   * @param encoderB     Encoder B reading.
   * @param minRotations Lowest shared gear angle in rotations.
   * @param maxRotations Highest shared gear angle in rotations.
   * @return The shared gear's angle, or empty if no single candidate matches.
   */
  private Optional<Angle> solve(Angle encoderA, Angle encoderB, double minRotations, double maxRotations) {
    double a = MathUtil.inputModulus(encoderA.in(Rotations), 0.0, 1.0);
    double b = MathUtil.inputModulus(encoderB.in(Rotations), 0.0, 1.0);
    if (!Double.isFinite(a) || !Double.isFinite(b)) {
      return Optional.empty();
    }

    // a = (ratioA * gear) mod 1, so gear = (a + turns) / ratioA for a whole number of turns.
    int minTurns = (int) Math.floor(ratioA * minRotations - a) - 1;
    int maxTurns = (int) Math.ceil(ratioA * maxRotations - a) + 1;

    double bestError = Double.MAX_VALUE;
    double secondError = Double.MAX_VALUE;
    double bestRotations = Double.NaN;
    for (int turns = minTurns; turns <= maxTurns; turns++) {
      double gearRotations = (a + turns) / ratioA;
      if (gearRotations < minRotations - 1e-6 || gearRotations > maxRotations + 1e-6) {
        continue;
      }
      double error = Math.abs(MathUtil.inputModulus(ratioB * gearRotations - b, -0.5, 0.5));
      if (error < bestError) {
        secondError = bestError;
        bestError = error;
        bestRotations = gearRotations;
      } else if (error < secondError) {
        secondError = error;
      }
    }

    // No candidate is close enough, or two are, which happens at the very ends of the range.
    if (bestError > toleranceRotations || secondError <= toleranceRotations) {
      return Optional.empty();
    }
    return Optional.of(Rotations.of(bestRotations));
  }

  /**
   * Greatest common divisor of two positive integers.
   *
   * @param a First integer.
   * @param b Second integer.
   * @return Greatest common divisor.
   */
  private static int gcd(int a, int b) {
    while (b != 0) {
      int remainder = a % b;
      a = b;
      b = remainder;
    }
    return a;
  }
}
