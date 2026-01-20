package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.RobotBase;
import java.math.BigInteger;
import java.util.Arrays;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

/**
 * Configuration for the CRT solver. Made by team 6911
 */
public class CRTAbsoluteEncoderConfig {

  private final Supplier<Angle> absoluteEncoder1AngleSupplier;
  private final Supplier<Angle> absoluteEncoder2AngleSupplier;

  // Directly-provided ratios (encoder rotations per mechanism rotation).
  private Optional<Double> encoder1RotPerMechanismRot = Optional.empty();
  private Optional<Double> encoder2RotPerMechanismRot = Optional.empty();

  // Optional encoder offsets applied to measurements before solving (added, then wrapped to [0,1)).
  private double absoluteEncoder1OffsetRot = 0.0;
  private double absoluteEncoder2OffsetRot = 0.0;

  // Allowed mechanism angle range in rotations.
  private double minMechanismRotations = 0.0;
  private double maxMechanismRotations = 1.0;

  // Maximum modular error allowed when matching predicted encoder 2 to measured encoder 2.
  private double matchToleranceRotations = 0.005;

  // Optional tooth/scale data for coverage calculation.
  private Optional<Integer> encoder1PrimeTeeth = Optional.empty();
  private Optional<Integer> encoder2PrimeTeeth = Optional.empty();
  private Optional<Double> commonScaleK = Optional.empty();

  // Optional CRT gear recommendation inputs.
  private Optional<Integer> gearSearchStage1GearTeeth = Optional.empty();
  private Optional<Double> gearSearchStage2Ratio = Optional.empty();
  private Optional<Double> gearSearchCoverageMargin = Optional.empty();
  private Optional<Integer> gearSearchMinTeeth = Optional.empty();
  private Optional<Integer> gearSearchMaxTeeth = Optional.empty();
  private Optional<Integer> gearSearchMaxIterations = Optional.empty();

  // Optional gearing definitions (for debug/telemetry, and as a fallback for ratio computation).
  private Optional<int[]> absoluteEncoder1TeethChain = Optional.empty();
  private Optional<int[]> absoluteEncoder2TeethChain = Optional.empty();
  private Optional<int[]> absoluteEncoder1TeethStages = Optional.empty();
  private Optional<int[]> absoluteEncoder2TeethStages = Optional.empty();

  private boolean encoder1Inverted = false;
  private boolean encoder2Inverted = false;

  public CRTAbsoluteEncoderConfig(
      Supplier<Angle> absoluteEncoder1AngleSupplier,
      Supplier<Angle> absoluteEncoder2AngleSupplier) {
    this.absoluteEncoder1AngleSupplier =
        Objects.requireNonNull(absoluteEncoder1AngleSupplier, "absoluteEncoder1AngleSupplier");
    this.absoluteEncoder2AngleSupplier =
        Objects.requireNonNull(absoluteEncoder2AngleSupplier, "absoluteEncoder2AngleSupplier");
  }

  /**
   * Sets encoder rotations per mechanism rotation directly for encoder 1 and encoder 2.
   *
   * <p>Use this when you already know the ratios and do not want to describe the gear train.
   * Positive ratios mean the encoder increases with positive mechanism rotation. If an encoder is
   * mounted in reverse, and you can't set inversion on-device, use {@link #withAbsoluteEncoder1Inverted(boolean)} or {@link
   * #withAbsoluteEncoder2Inverted(boolean)} to flip the sign.
   */
  public CRTAbsoluteEncoderConfig withEncoderRatios(
      double encoder1RotPerMechanismRot, double encoder2RotPerMechanismRot) {
    this.encoder1RotPerMechanismRot = Optional.of(encoder1RotPerMechanismRot);
    this.encoder2RotPerMechanismRot = Optional.of(encoder2RotPerMechanismRot);
    return this;
  }

  /**
   * Sets ratios using a shared drive gear stage.
   *
   * <p>Formula per encoder: ratio = commonRatio * (driveGearTeeth / encoderPinionTeeth). This
   * assumes both encoders are driven off the same drive gear after the shared reduction.
   *
   * <p>Example: turret gearbox is 12:50 -> 10:110 and the encoders are driven by 30t and 31t gears
   * The common ratio is 110/10, the drive gear is 50T. 
   * 
   * <p> If the encoder gears are driven by the turret gear
   * itself, the common ratio is 1, and thed drive gear is the turret gear teeth
   *
   * <p>Also seeds CRT gear recommendation inputs (stage1 gear teeth + stage2 ratio) so you can
   * call {@link #withCrtGearRecommendationConstraints(double, int, int, int)} afterward.
   */
  public CRTAbsoluteEncoderConfig withCommonDriveGear(
      double commonRatio,
      int driveGearTeeth,
      int absoluteEncoder1PinionTeeth,
      int absoluteEncoder2PinionTeeth) {

    requireNonZeroFinite(commonRatio, "commonRatio");
    requirePositiveTeeth(driveGearTeeth, "EncoderDriveGearTeeth");
    requirePositiveTeeth(absoluteEncoder1PinionTeeth, "Encoder1GearTeeth");
    requirePositiveTeeth(absoluteEncoder2PinionTeeth, "Encoder2GearTeeth");

    double ratio1 = ratioFromCommonDrive(commonRatio, driveGearTeeth, absoluteEncoder1PinionTeeth);
    double ratio2 = ratioFromCommonDrive(commonRatio, driveGearTeeth, absoluteEncoder2PinionTeeth);
    withEncoderRatios(ratio1, ratio2);

    this.encoder1PrimeTeeth = Optional.of(absoluteEncoder1PinionTeeth);
    this.encoder2PrimeTeeth = Optional.of(absoluteEncoder2PinionTeeth);
    this.commonScaleK = Optional.of(commonRatio * driveGearTeeth);
    this.gearSearchStage1GearTeeth = Optional.of(driveGearTeeth);
    this.gearSearchStage2Ratio = Optional.of(commonRatio);
    return this;
  }

  /**
   * Sets encoder offsets added to raw absolute readings before wrap (in rotations).
   *
   * <p>If your encoder supports on device zeroing/offsets, configure it there
   * and keep these offsets at zero to avoid double-offsetting.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoderOffsets(
      Angle encoder1Offset, Angle encoder2Offset) {
    this.absoluteEncoder1OffsetRot =
        Objects.requireNonNull(encoder1Offset, "encoder1Offset").in(Rotations);
    this.absoluteEncoder2OffsetRot =
        Objects.requireNonNull(encoder2Offset, "encoder2Offset").in(Rotations);
    return this;
  }

  /** Sets the allowed mechanism angle range (in rotations). */
  public CRTAbsoluteEncoderConfig withMechanismRange(Angle minAngle, Angle maxAngle) {
    this.minMechanismRotations = Objects.requireNonNull(minAngle, "minAngle").in(Rotations);
    this.maxMechanismRotations = Objects.requireNonNull(maxAngle, "maxAngle").in(Rotations);
    return this;
  }

  /**
   * Sets the match tolerance for encoder 2 modular error (in rotations).
   *
   * <p>Lower values are reasonable when backlash and sensor noise 
   * are low, but too small can yield no solution; high backlash mechanisms may need larger values. 
   * When backlash is the main source of error and not noise, the tolerance can often be close to the 
   * backlash.
   * 
   * <p>Experimenting with always pushing the mechanism to one side of the backlash can allow a
   * reduced tolerance, but teams should validate this to ensure it will always solve with their
   * specified tolerance. Tune based on {@code getLastErrorRotations()}.
   *
   * <p>Example: mechanism backlash to the encoders is about 1 degree. commonRatio = 11, driveGearTeeth = 50, and
   * encoder2PinionTeeth = 30. Ratio = 18.3333. 1 degree of mechanism backlash is about 0.0509
   * rotations of the encoder. A tolerance of 0.06 rotations corresponds to about 1.18 degrees
   * at the mechanism (0.06 / 18.33333 * 360), which is only slightly higher than the backlash.
   */
  public CRTAbsoluteEncoderConfig withMatchTolerance(Angle tolerance) {
    this.matchToleranceRotations = Objects.requireNonNull(tolerance, "tolerance").in(Rotations);
    return this;
  }

  /**
   * If true, flips encoder 1 ratio sign (use when the sensor is mounted reversed).
   *
   * <p>If your encoder supports on-device direction/inversion, configure it
   * there and keep this false to avoid double-inversion.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder1Inverted(boolean inverted) {
    this.encoder1Inverted = inverted;
    return this;
  }

  /**
   * If true, flips encoder 2 ratio sign (use when the sensor is mounted reversed).
   *
   * <p>If your encoder supports on-device direction/inversion, configure it
   * there and keep this false to avoid double-inversion.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder2Inverted(boolean inverted) {
    this.encoder2Inverted = inverted;
    return this;
  }

  /**
   * Sets the stage1 gear teeth and stage2 ratio inputs used for gear recommendations.
   *
   * <p>Call {@link #withCrtGearRecommendationConstraints(double, int, int, int)} to configure
   * coverage and search bounds.
   *
   * <p>{@code stage1GearTeeth} is the gear that drives both encoder pinions, and {@code stage2Ratio}
   * is the shared "common ratio" between the mechanism and that drive gear.
   */
  public CRTAbsoluteEncoderConfig withCrtGearRecommendationInputs(
      int stage1GearTeeth, double stage2Ratio) {
    requirePositiveTeeth(stage1GearTeeth, "stage1GearTeeth");
    requireNonZeroFinite(stage2Ratio, "stage2Ratio");
    this.gearSearchStage1GearTeeth = Optional.of(stage1GearTeeth);
    this.gearSearchStage2Ratio = Optional.of(stage2Ratio);
    return this;
  }

  /**
   * Sets CRT gear recommendation constraints (coverage margin + search bounds).
   *
   * <p>Call this after {@link #withCommonDriveGear(double, int, int, int)} or {@link
   * #withCrtGearRecommendationInputs(int, double)}.
   *
   * <p>No-op when not running in simulation to avoid extraneous calculations on a real robot.
   */
  public CRTAbsoluteEncoderConfig withCrtGearRecommendationConstraints(
      double coverageMargin, int minTeeth, int maxTeeth, int maxIterationsLimit) {
    if (!RobotBase.isSimulation()) {
      return this;
    }
    requirePositiveFinite(coverageMargin, "coverageMargin");
    requirePositiveTeeth(minTeeth, "minTeeth");
    requirePositiveTeeth(maxTeeth, "maxTeeth");
    if (maxTeeth < minTeeth) {
      throw new IllegalArgumentException("maxTeeth must be >= minTeeth");
    }
    if (maxIterationsLimit < 1) {
      throw new IllegalArgumentException("maxIterationsLimit must be >= 1");
    }
    this.gearSearchCoverageMargin = Optional.of(coverageMargin);
    this.gearSearchMinTeeth = Optional.of(minTeeth);
    this.gearSearchMaxTeeth = Optional.of(maxTeeth);
    this.gearSearchMaxIterations = Optional.of(maxIterationsLimit);
    return this;
  }


  // --- Gearing helpers ---

  /**
   * Defines a meshed gear chain for encoder 1, ordered from mechanism drive gear to encoder.
   *
   * <p>Example: {@code withAbsoluteEncoder1Gearing(50, 20, 40)} means 50T drives 20T, which drives
   * 40T on encoder 1, for a ratio of (50/20) * (20/40) = 50/40. A simpler one-stage chain would be
   * {@code withAbsoluteEncoder1Gearing(72, 24)} for a 3:1 reduction.
   *
   * <p>Not valid for compound same-shaft trains; use {@link
   * #withAbsoluteEncoder1GearingStages(int...)} instead.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder1Gearing(int... teethChain) {
    this.absoluteEncoder1TeethChain =
        Optional.of(copyTeeth(teethChain, "absoluteEncoder1TeethChain"));
    return this;
  }

  /**
   * Defines a meshed gear chain for encoder 2, ordered from mechanism drive gear to encoder.
   *
   * <p>Example: {@code withAbsoluteEncoder2Gearing(50, 20, 40)} means 50T drives 20T, which drives
   * 40T on encoder 2, for a ratio of (50/20) * (20/40) = 50/40. A single mesh could be
   * {@code withAbsoluteEncoder2Gearing(60, 20)} for a 3:1 reduction.
   *
   * <p>Not valid for compound same-shaft trains; use {@link
   * #withAbsoluteEncoder2GearingStages(int...)} instead.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder2Gearing(int... teethChain) {
    this.absoluteEncoder2TeethChain =
        Optional.of(copyTeeth(teethChain, "absoluteEncoder2TeethChain"));
    return this;
  }

  /**
   * Defines explicit mesh stages for encoder 1 as (driverTeeth, drivenTeeth) pairs.
   *
   * <p>Use this for compound trains or when you need same-shaft gears represented by separate
   * stages. Example: {@code withAbsoluteEncoder1GearingStages(12, 36, 18, 60)} means 12T drives
   * 36T, then 18T drives 60T. A single stage would be {@code withAbsoluteEncoder1GearingStages(12,
   * 60)}.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder1GearingStages(int... driverDrivenPairs) {
    this.absoluteEncoder1TeethStages =
        Optional.of(copyTeeth(driverDrivenPairs, "absoluteEncoder1TeethStages"));
    return this;
  }

  /**
   * Defines explicit mesh stages for encoder 2 as (driverTeeth, drivenTeeth) pairs.
   *
   * <p>Use this for compound trains or when you need same-shaft gears represented by separate
   * stages. Example: {@code withAbsoluteEncoder2GearingStages(12, 36, 18, 60)} means 12T drives
   * 36T, then 18T drives 60T.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder2GearingStages(int... driverDrivenPairs) {
    this.absoluteEncoder2TeethStages =
        Optional.of(copyTeeth(driverDrivenPairs, "absoluteEncoder2TeethStages"));
    return this;
  }

  /** Builds a MechanismGearing for encoder 1 from the configured chain/stages. */
  public MechanismGearing getAbsoluteEncoder1Gearing() {
    return buildMechanismGearingForEncoder(1);
  }

  /** Builds a MechanismGearing for encoder 2 from the configured chain/stages. */
  public MechanismGearing getAbsoluteEncoder2Gearing() {
    return buildMechanismGearingForEncoder(2);
  }

  private MechanismGearing buildMechanismGearingForEncoder(int encoderIndex) {
    Optional<int[]> chain =
        (encoderIndex == 1) ? absoluteEncoder1TeethChain : absoluteEncoder2TeethChain;
    Optional<int[]> pairs =
        (encoderIndex == 1) ? absoluteEncoder1TeethStages : absoluteEncoder2TeethStages;

    if (pairs.isPresent()) {
      int[] p = pairs.get();
      if (p.length < 2 || (p.length % 2) != 0) {
        throw new IllegalStateException(
            "Encoder "
                + encoderIndex
                + " gear stages must be (driver,driven) pairs (even length >= 2).");
      }
      validatePositiveTeeth(p, "encoder " + encoderIndex + " gear stages");
      return new MechanismGearing(GearBox.fromStages(buildStagesFromDriverDrivenPairs(p)));
    }

    if (chain.isPresent()) {
      int[] t = chain.get();
      if (t.length < 2) {
        throw new IllegalStateException(
            "Encoder " + encoderIndex + " gear chain must have >= 2 tooth counts.");
      }
      validatePositiveTeeth(t, "encoder " + encoderIndex + " gear chain");
      return new MechanismGearing(GearBox.fromStages(buildStagesFromChain(t)));
    }

    throw new IllegalStateException("Absolute encoder " + encoderIndex + " gearing not set.");
  }

  // --- Getters used by CRTAbsoluteEncoder ---

  public Angle getAbsoluteEncoder1Angle() {
    Angle value = absoluteEncoder1AngleSupplier.get();
    return value != null ? value : Rotations.of(Double.NaN);
  }

  public Angle getAbsoluteEncoder2Angle() {
    Angle value = absoluteEncoder2AngleSupplier.get();
    return value != null ? value : Rotations.of(Double.NaN);
  }

  public double getAbsoluteEncoder1OffsetRotations() {
    return absoluteEncoder1OffsetRot;
  }

  public double getAbsoluteEncoder2OffsetRotations() {
    return absoluteEncoder2OffsetRot;
  }

  public double getMinMechanismRotations() {
    return minMechanismRotations;
  }

  public double getMaxMechanismRotations() {
    return maxMechanismRotations;
  }

  /** Allowed mechanism travel in rotations (max - min). */
  public double getMechanismRange() {
    return maxMechanismRotations - minMechanismRotations;
  }

  public double getMatchToleranceRotations() {
    return matchToleranceRotations;
  }

  /** Returns encoder 1 rotations per mechanism rotation (includes inversion if enabled). */
  public double getEncoder1RotationsPerMechanismRotation() {
    double ratio = getOrComputeRatio(1);
    return encoder1Inverted ? -ratio : ratio;
  }

  /** Returns encoder 2 rotations per mechanism rotation (includes inversion if enabled). */
  public double getEncoder2RotationsPerMechanismRotation() {
    double ratio = getOrComputeRatio(2);
    return encoder2Inverted ? -ratio : ratio;
  }

  private double getOrComputeRatio(int encoderIndex) {
    if (encoderIndex == 1 && encoder1RotPerMechanismRot.isPresent()) {
      return encoder1RotPerMechanismRot.get();
    }
    if (encoderIndex == 2 && encoder2RotPerMechanismRot.isPresent()) {
      return encoder2RotPerMechanismRot.get();
    }

    Optional<int[]> pairs =
        (encoderIndex == 1) ? absoluteEncoder1TeethStages : absoluteEncoder2TeethStages;
    Optional<int[]> chain =
        (encoderIndex == 1) ? absoluteEncoder1TeethChain : absoluteEncoder2TeethChain;
    if (pairs.isPresent()) {
      return ratioFromDriverDrivenPairs(pairs.get());
    }
    if (chain.isPresent()) {
      return ratioFromChain(chain.get());
    }

    throw new IllegalStateException(
        "Encoder ratios not configured. Use withEncoderRatios(...) or provide gearing.");
  }

  /**
   * Computes ratio from a simple meshed gear chain (driver/gear/.../encoder).
   *
   * <p>Returns encoder rotations per mechanism rotation.
   */
  public static double ratioFromChain(int... teethChain) {
    String[] stages = buildStagesFromChain(teethChain);
    return new MechanismGearing(GearBox.fromStages(stages)).getMechanismToRotorRatio();
  }

  /**
   * Computes ratio from explicit (driver, driven) stage pairs.
   *
   * <p>Returns encoder rotations per mechanism rotation.
   */
  public static double ratioFromDriverDrivenPairs(int... driverDrivenPairs) {
    String[] stages = buildStagesFromDriverDrivenPairs(driverDrivenPairs);
    return new MechanismGearing(GearBox.fromStages(stages)).getMechanismToRotorRatio();
  }

  /**
   * Computes ratio from a shared drive stage: commonRatio * (driveGear / encoderGear).
   *
   * <p>Returns encoder rotations per mechanism rotation.
   */
  public static double ratioFromCommonDrive(
      double commonRatio, int driveGearTeeth, int encoderTeeth) {
    requireNonZeroFinite(commonRatio, "commonRatio");
    requirePositiveTeeth(driveGearTeeth, "driveGearTeeth");
    requirePositiveTeeth(encoderTeeth, "encoderTeeth");
    return commonRatio * (((double) driveGearTeeth) / encoderTeeth);
  }

  /**
   * Returns unique coverage in mechanism rotations, if prime teeth + common scale are set.
   *
   */
  public Optional<Double> getUniqueCoverageRotations() {
    if (encoder1PrimeTeeth.isEmpty() || encoder2PrimeTeeth.isEmpty() || commonScaleK.isEmpty()) {
      return Optional.empty();
    }
    double k = commonScaleK.get();
    if (!Double.isFinite(k) || Math.abs(k) < 1e-12) {
      return Optional.empty();
    }
    int l = lcm(encoder1PrimeTeeth.get(), encoder2PrimeTeeth.get());
    return Optional.of(l / k);
  }

  public static boolean isCoprime(int a, int b) {
    return BigInteger.valueOf(a).gcd(BigInteger.valueOf(b)).equals(BigInteger.ONE);
  }

  /**
   * Returns a recommended CRT gear pair if gear-search inputs are configured.
   */
  public Optional<CrtGearPair> getRecommendedCrtGearPair() {
    if (gearSearchStage1GearTeeth.isEmpty()
        || gearSearchStage2Ratio.isEmpty()
        || gearSearchCoverageMargin.isEmpty()
        || gearSearchMinTeeth.isEmpty()
        || gearSearchMaxTeeth.isEmpty()
        || gearSearchMaxIterations.isEmpty()) {
      return Optional.empty();
    }

    if (!Double.isFinite(maxMechanismRotations) || maxMechanismRotations <= 0.0) {
      return Optional.empty();
    }

    CrtGearPair pair =
        findSmallestCrtGearPair(
            gearSearchStage1GearTeeth.get(),
            gearSearchStage2Ratio.get(),
            maxMechanismRotations,
            gearSearchCoverageMargin.get(),
            gearSearchMinTeeth.get(),
            gearSearchMaxTeeth.get(),
            gearSearchMaxIterations.get());
    return Optional.ofNullable(pair);
  }

  /**
   * Finds the smallest gear pair that satisfies coverage and iteration limits.
   */
  public static CrtGearPair findSmallestCrtGearPair(
      int stage1GearTeeth,
      double stage2Ratio,
      double maxMechanismRotations,
      double coverageMargin,
      int minTeeth,
      int maxTeeth,
      int maxIterationsLimit) {
    if (stage1GearTeeth <= 0 || stage2Ratio <= 0.0 || minTeeth < 1 || maxTeeth < minTeeth) {
      return null;
    }

    double requiredCoverageRot = maxMechanismRotations * coverageMargin;
    int requiredLcm = (int) Math.ceil(requiredCoverageRot * stage2Ratio * stage1GearTeeth - 1e-9);
    CrtGearPair best = null;
    int bestMaxTeeth = Integer.MAX_VALUE;
    int bestSumTeeth = Integer.MAX_VALUE;
    int bestLcm = Integer.MAX_VALUE;

    for (int a = minTeeth; a <= maxTeeth; a++) {
      for (int b = a + 1; b <= maxTeeth; b++) {
        int lcm = lcm(a, b);
        if (lcm < requiredLcm) {
          continue;
        }
        int iterationsA =
            theoreticalIterationsForGear(a, stage1GearTeeth, stage2Ratio, maxMechanismRotations);
        int iterationsB =
            theoreticalIterationsForGear(b, stage1GearTeeth, stage2Ratio, maxMechanismRotations);
        boolean aOk = iterationsA <= maxIterationsLimit;
        boolean bOk = iterationsB <= maxIterationsLimit;
        if (!aOk && !bOk) {
          continue;
        }
        int assignedA = a;
        int assignedB = b;
        int assignedIterations = iterationsA;
        if (bOk && (!aOk || iterationsB < iterationsA)) {
          assignedA = b;
          assignedB = a;
          assignedIterations = iterationsB;
        }
        int candidateMaxTeeth = b;
        int sumTeeth = a + b;
        if (candidateMaxTeeth < bestMaxTeeth
            || (candidateMaxTeeth == bestMaxTeeth && sumTeeth < bestSumTeeth)
            || (candidateMaxTeeth == bestMaxTeeth && sumTeeth == bestSumTeeth && lcm < bestLcm)) {
          double coverageRot = lcm / (stage2Ratio * stage1GearTeeth);
          bestMaxTeeth = candidateMaxTeeth;
          bestSumTeeth = sumTeeth;
          bestLcm = lcm;
          best =
              new CrtGearPair(
                  assignedA, assignedB, lcm, coverageRot, gcd(a, b), assignedIterations);
        }
      }
    }

    return best;
  }

  private static int theoreticalIterationsForGear(
      int gearTeeth, int stage1GearTeeth, double stage2Ratio, double maxMechanismRotations) {
    double ratioA = stage2Ratio * ((double) stage1GearTeeth / gearTeeth);
    return (int) Math.ceil(ratioA * maxMechanismRotations) + 3;
  }

  private static void requireNonZeroFinite(double value, String label) {
    if (!Double.isFinite(value) || Math.abs(value) < 1e-12) {
      throw new IllegalArgumentException(label + " must be finite and non-zero");
    }
  }

  private static void requirePositiveTeeth(int value, String label) {
    if (value <= 0) {
      throw new IllegalArgumentException(label + " must be > 0");
    }
  }

  private static void requirePositiveFinite(double value, String label) {
    if (!Double.isFinite(value) || value <= 0.0) {
      throw new IllegalArgumentException(label + " must be finite and > 0");
    }
  }

  private static void validatePositiveTeeth(int[] values, String label) {
    for (int value : values) {
      if (value <= 0) {
        throw new IllegalArgumentException(label + " must be > 0");
      }
    }
  }

  /**
   * Converts a driver-to-driven tooth chain into GearBox stage strings ("driver:driven").
   *
   * <p>Example: {@code [50, 20, 40]} becomes {@code ["50:20", "20:40"]}.
   */
  private static String[] buildStagesFromChain(int[] teethChain) {
    Objects.requireNonNull(teethChain, "teethChain");
    if (teethChain.length < 2) {
      throw new IllegalArgumentException("Gear chain must have >= 2 tooth counts");
    }
    validatePositiveTeeth(teethChain, "gear chain");
    String[] stages = new String[teethChain.length - 1];
    for (int i = 1; i < teethChain.length; i++) {
      stages[i - 1] = teethChain[i - 1] + ":" + teethChain[i];
    }
    return stages;
  }

  /**
   * Converts (driver, driven) stage pairs into GearBox stage strings ("driver:driven").
   *
   * <p>Example: {@code [12, 36, 18, 60]} becomes {@code ["12:36", "18:60"]}.
   */
  private static String[] buildStagesFromDriverDrivenPairs(int[] driverDrivenPairs) {
    Objects.requireNonNull(driverDrivenPairs, "driverDrivenPairs");
    if (driverDrivenPairs.length < 2 || (driverDrivenPairs.length % 2) != 0) {
      throw new IllegalArgumentException("Stages must be (driver,driven) pairs (even length >= 2)");
    }
    validatePositiveTeeth(driverDrivenPairs, "driver/driven stages");
    String[] stages = new String[driverDrivenPairs.length / 2];
    int idx = 0;
    for (int i = 0; i < driverDrivenPairs.length; i += 2) {
      int driver = driverDrivenPairs[i];
      int driven = driverDrivenPairs[i + 1];
      stages[idx++] = driver + ":" + driven;
    }
    return stages;
  }

  private static int[] copyTeeth(int[] teeth, String label) {
    Objects.requireNonNull(teeth, label);
    return Arrays.copyOf(teeth, teeth.length);
  }

  private static int gcd(int a, int b) {
    int x = Math.abs(a), y = Math.abs(b);
    while (y != 0) {
      int t = y;
      y = x % y;
      x = t;
    }
    return x;
  }

  private static int lcm(int a, int b) {
    return (a / gcd(a, b)) * b;
  }

  public static record CrtGearPair(
      int gearA, int gearB, int lcm, double coverageRot, int gcd, int theoreticalIterations) {}
}
