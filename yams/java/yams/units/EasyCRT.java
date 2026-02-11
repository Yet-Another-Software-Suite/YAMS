package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
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
 * 
 * <p>Created by team 6911.
 */
public class EasyCRT {
  /**
   * The list of possible statuses from the previous solve attempt.
   */
  public enum CRTStatus {
    /**
     * The previous solve succeeded.
     */
    OK,
    /**
     * The previous solve attempt could not find a solution.
     */
    NO_SOLUTION,
    /**
     * The previous solve attempt resulted in two nearly-equal matches within tolerance.
     */
    AMBIGUOUS,
    /**
     * No solve attempts have occured.
     */
    NOT_ATTEMPTED,
    /**
     * A solve was not attempted, as the solver was not configured correctly.
     */
    INVALID_CONFIG
  }

  /**
   * Configuration containing ratios, limits, and encoder suppliers.
   */
  private final EasyCRTConfig easyCrtConfig;

  /**
   * Last solve status string (e.g., OK / NO_SOLUTION / AMBIGUOUS / INVALID_CONFIG).
   */
  private CRTStatus lastStatus = CRTStatus.NOT_ATTEMPTED;

  /**
   * Last best-match modular error in rotations.
   */
  private double lastErrorRot = Double.NaN;

  /**
   * Number of candidates evaluated in the most recent solve attempt.
   */
  private int lastIterations = 0;

  /**
   * Creates an EasyCRT solver.
   *
   * @param easyCrtConfig configuration describing encoder ratios, offsets, and limits
   */
  public EasyCRT(EasyCRTConfig easyCrtConfig) {
    this.easyCrtConfig = easyCrtConfig;
  }

  /**
   * Returns the mechanism angle if a unique solution is found.
   *
   * <p>If no unique solution is found (outside tolerance or ambiguous), returns {@link
   * Optional#empty()}.
   *
   * @return optional containing mechanism angle when uniquely resolved
   */
  public Optional<Angle> getAngleOptional() {
    final double ratio1 = easyCrtConfig.getEncoder1RotationsPerMechanismRotation();
    final double ratio2 = easyCrtConfig.getEncoder2RotationsPerMechanismRotation();
    final double minMechRot = easyCrtConfig.getMinMechanismAngle().in(Rotations);
    final double maxMechRot = easyCrtConfig.getMaxMechanismAngle().in(Rotations);
    final double tolRot = easyCrtConfig.getMatchTolerance().in(Rotations);

    // Read + wrap into [0, 1).
    final double abs1 =
        MathUtil.inputModulus(
            easyCrtConfig.getAbsoluteEncoder1Angle()
                .plus(easyCrtConfig.getAbsoluteEncoder1Offset())
                .in(Rotations),
            0.0,
            1.0);
    final double abs2 =
        MathUtil.inputModulus(
            easyCrtConfig.getAbsoluteEncoder2Angle()
                .plus(easyCrtConfig.getAbsoluteEncoder2Offset())
                .in(Rotations),
            0.0,
            1.0);

    CrtSolution sol =
        resolveFromSensors(abs1, abs2, ratio1, ratio2, minMechRot, maxMechRot, tolRot);

    if (sol == null) {
      return Optional.empty();
    }
    return Optional.of(Rotations.of(sol.mechanismRotations()));
  }

  /**
   * Returns the last solver status string.
   *
   * @return status from the previous solve attempt
   */
  public CRTStatus getLastStatus() {
    return lastStatus;
  }

  /**
   * Returns the last best-match modular error in rotations.
   *
   * @return last modular error, or NaN if not solved
   */
  public double getLastErrorRotations() {
    return lastErrorRot;
  }

  /**
   * Returns the number of candidates evaluated in the last solve attempt.
   *
   * @return iteration count from the previous solve attempt
   */
  public int getLastIterations() {
    return lastIterations;
  }

  /**
   * Solves for mechanism rotations using wrapped encoder readings and configured ratios.
   *
   * @param abs1 wrapped absolute encoder 1 reading in rotations
   * @param abs2 wrapped absolute encoder 2 reading in rotations
   * @param ratio1 encoder 1 rotations per mechanism rotation
   * @param ratio2 encoder 2 rotations per mechanism rotation
   * @param minMechanismRotations minimum allowed mechanism rotations
   * @param maxMechanismRotations maximum allowed mechanism rotations
   * @param matchTolerance maximum allowed modular error to accept a solution
   * @return solution containing mechanism rotations and error, or null when not found or ambiguous
   */
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
      lastStatus = CRTStatus.INVALID_CONFIG;
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

      double predicted2 = MathUtil.inputModulus(ratio2 * mechRot, 0.0, 1.0);
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
      lastStatus = CRTStatus.NO_SOLUTION;
      lastErrorRot = bestErr;
      return null;
    }

    // If there are two nearly-equal matches within tolerance, the solution is ambiguous.
    if (secondErr <= matchTolerance && Math.abs(secondErr - bestErr) < 1e-3) {
      lastStatus = CRTStatus.AMBIGUOUS;
      lastErrorRot = bestErr;
      return null;
    }

    lastStatus = CRTStatus.OK;
    lastErrorRot = bestErr;
    return new CrtSolution(bestRot, bestErr);
  }

  /**
   * Computes the minimal modular difference between two wrapped values.
   *
   * @param a first wrapped value
   * @param b second wrapped value
   * @return minimal modular error in rotations
   */
  private static double modularError(double a, double b) {
    double diff = Math.abs(a - b);
    return diff > 0.5 ? 1.0 - diff : diff;
  }

  /**
   * Container for a CRT solve result.
   *
   * @param mechanismRotations solved mechanism rotations
   * @param errorRotations modular error associated with the solution
   */
  private static record CrtSolution(double mechanismRotations, double errorRotations) {}
}
