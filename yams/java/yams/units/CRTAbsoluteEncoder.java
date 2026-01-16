package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import java.util.Optional;

/**
 * A CRT style absolute mechanism angle estimator using two absolute encoders.
 *
 * <ol>
 *   <li>Generate all mechanism angle candidates consistent with encoder 1 within the allowed range.
 *   <li>For each candidate, predict what encoder 2 should read and score the modular error.
 *   <li>Select the best unique match within a configurable tolerance.
 * </ol>
 *
 * <p>This is <b>not</b> a textbook Chinese Remainder Theorem solve; it is a
 * "CRT-inspired" unwrapping method that is easier to keep stable with backlash and sensor
 * noise.
 */
public class CRTAbsoluteEncoder {

  private final CRTAbsoluteEncoderConfig config;

  private String lastStatus = "NOT_ATTEMPTED";
  private double lastErrorRot = Double.NaN;
  private int lastIterations = 0;

  public CRTAbsoluteEncoder(CRTAbsoluteEncoderConfig cfg) {
    this.config = cfg;
  }

  /**
   * Returns the mechanism angle if a unique solution is found.
   *
   * <p>If no unique solution is found (outside tolerance or ambiguous), returns {@link
   * Optional#empty()}.
   */
  public Optional<Angle> getAngleOptional() {
    final double ratio1 = config.getEncoder1RotationsPerMechanismRotation();
    final double ratio2 = config.getEncoder2RotationsPerMechanismRotation();
    final double minMechRot = config.getMinMechanismRotations();
    final double maxMechRot = config.getMaxMechanismRotations();
    final double tolRot = config.getMatchToleranceRotations();

    // Read + wrap into [0, 1).
    final double abs1 =
        wrap01(
            config.getAbsoluteEncoder1Angle().in(Rotations)
                + config.getAbsoluteEncoder1OffsetRotations());
    final double abs2 =
        wrap01(
            config.getAbsoluteEncoder2Angle().in(Rotations)
                + config.getAbsoluteEncoder2OffsetRotations());

    CrtSolution sol =
        resolveFromSensors(abs1, abs2, ratio1, ratio2, minMechRot, maxMechRot, tolRot);

    if (sol == null) {
      return Optional.empty();
    }
    return Optional.of(Rotations.of(sol.mechanismRotations()));
  }

  /** Last solver status string (e.g., OK / NO_SOLUTION / AMBIGUOUS / INVALID_CONFIG). */
  public String getLastStatus() {
    return lastStatus;
  }

  /** Last best-match modular error in rotations. */
  public double getLastErrorRotations() {
    return lastErrorRot;
  }

  /** Number of candidates evaluated in the last solve attempt. */
  public int getLastIterations() {
    return lastIterations;
  }

  private CrtSolution resolveFromSensors(
      double abs1,
      double abs2,
      double ratio1,
      double ratio2,
      double minMechanismRotations,
      double maxMechanismRotations,
      double matchTolerance) {

    lastIterations = 0;
    lastErrorRot = Double.NaN;

    if (!Double.isFinite(abs1)
        || !Double.isFinite(abs2)
        || !Double.isFinite(ratio1)
        || !Double.isFinite(ratio2)
        || Math.abs(ratio1) < 1e-12
        || !Double.isFinite(minMechanismRotations)
        || !Double.isFinite(maxMechanismRotations)
        || minMechanismRotations > maxMechanismRotations
        || !Double.isFinite(matchTolerance)
        || matchTolerance < 0.0) {
      lastStatus = "INVALID_CONFIG";
      return null;
    }

    double bestErr = Double.MAX_VALUE;
    double secondErr = Double.MAX_VALUE;
    double bestRot = Double.NaN;

    // Derive integer wrap-count bounds from the allowed mechanism angle range.
    //
    // abs1 = (ratio1 * mechRot) mod 1
    // ratio1 * mechRot = abs1 + n, where n is an integer.
    // mechRot = (abs1 + n) / ratio1
    //
    // For mechRot within [min, max], n lies within [ratio1*min - abs1, ratio1*max - abs1]
    // (endpoints swap if ratio1 is negative).
    double nMinD = Math.min(ratio1 * minMechanismRotations, ratio1 * maxMechanismRotations) - abs1;
    double nMaxD = Math.max(ratio1 * minMechanismRotations, ratio1 * maxMechanismRotations) - abs1;
    int minN = (int) Math.floor(nMinD) - 1;
    int maxN = (int) Math.ceil(nMaxD) + 1;

    for (int n = minN; n <= maxN; n++) {
      lastIterations++;

      double mechRot = (abs1 + n) / ratio1;
      if (mechRot < minMechanismRotations - 1e-6 || mechRot > maxMechanismRotations + 1e-6) {
        continue;
      }

      double predicted2 = wrap01(ratio2 * mechRot);
      double err = modularError(predicted2, abs2);

      if (err < bestErr) {
        secondErr = bestErr;
        bestErr = err;
        bestRot = mechRot;
      } else if (err < secondErr) {
        secondErr = err;
      }
    }

    if (!Double.isFinite(bestRot) || bestErr > matchTolerance) {
      lastStatus = "NO_SOLUTION";
      lastErrorRot = bestErr;
      return null;
    }

    // If there are two nearly-equal matches within tolerance, the solution is ambiguous.
    if (secondErr <= matchTolerance && Math.abs(secondErr - bestErr) < 1e-3) {
      lastStatus = "AMBIGUOUS";
      lastErrorRot = bestErr;
      return null;
    }

    lastStatus = "OK";
    lastErrorRot = bestErr;
    return new CrtSolution(bestRot, bestErr);
  }

  private static double wrap01(double rotations) {
    double wrapped = rotations % 1.0;
    if (wrapped < 0.0) {
      wrapped += 1.0;
    }
    return wrapped;
  }

  private static double modularError(double a, double b) {
    double diff = Math.abs(a - b);
    return diff > 0.5 ? 1.0 - diff : diff;
  }

  private static record CrtSolution(double mechanismRotations, double errorRotations) {}
}
