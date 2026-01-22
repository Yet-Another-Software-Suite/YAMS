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
 * Configuration for the EasyCRT solver. Made by team 6911.
 */
public class EasyCRTConfig {

  /**
   * Supplies the absolute angle measurement for encoder 1.
   */
  private final Supplier<Angle> absoluteEncoder1AngleSupplier;

  /**
   * Supplies the absolute angle measurement for encoder 2.
   */
  private final Supplier<Angle> absoluteEncoder2AngleSupplier;

  /**
   * Directly provided rotations per mechanism rotation for encoder 1.
   */
  private Optional<Double> encoder1RotPerMechanismRot = Optional.empty();

  /**
   * Directly provided rotations per mechanism rotation for encoder 2.
   */
  private Optional<Double> encoder2RotPerMechanismRot = Optional.empty();

  /**
   * Offset in rotations added to encoder 1 before wrap.
   */
  private Angle absoluteEncoder1Offset = Rotations.of(0.0);

  /**
   * Offset in rotations added to encoder 2 before wrap.
   */
  private Angle absoluteEncoder2Offset = Rotations.of(0.0);

  /**
   * Minimum allowed mechanism rotation.
   */
  private Angle minMechanismAngle = Rotations.of(0.0);

  /**
   * Maximum allowed mechanism rotation.
   */
  private Angle maxMechanismAngle = Rotations.of(1.0);

  /**
   * Maximum modular error allowed when matching encoder 2.
   */
  private Angle matchTolerance = Rotations.of(0.005);

  /**
   * Optional prime tooth count for encoder 1 gear.
   */
  private Optional<Integer> encoder1PrimeTeeth = Optional.empty();

  /**
   * Optional prime tooth count for encoder 2 gear.
   */
  private Optional<Integer> encoder2PrimeTeeth = Optional.empty();

  /**
   * Optional common scale constant k for coverage calculations.
   */
  private Optional<Double> commonScaleK = Optional.empty();

  /**
   * Optional stage 1 gear teeth for CRT recommendations.
   */
  private Optional<Integer> gearSearchStage1GearTeeth = Optional.empty();

  /**
   * Optional stage 2 ratio for CRT recommendations.
   */
  private Optional<Double> gearSearchStage2Ratio = Optional.empty();

  /**
   * Optional coverage margin for CRT recommendations.
   */
  private Optional<Double> gearSearchCoverageMargin = Optional.empty();

  /**
   * Optional minimum tooth count to search for CRT recommendations.
   */
  private Optional<Integer> gearSearchMinTeeth = Optional.empty();

  /**
   * Optional maximum tooth count to search for CRT recommendations.
   */
  private Optional<Integer> gearSearchMaxTeeth = Optional.empty();

  /**
   * Optional maximum iterations to search for CRT recommendations.
   */
  private Optional<Integer> gearSearchMaxIterations = Optional.empty();

  /**
   * Optional meshed tooth chain for encoder 1 gearing.
   */
  private Optional<int[]> absoluteEncoder1TeethChain = Optional.empty();

  /**
   * Optional meshed tooth chain for encoder 2 gearing.
   */
  private Optional<int[]> absoluteEncoder2TeethChain = Optional.empty();

  /**
   * Optional (driver, driven) stage pairs for encoder 1 gearing.
   */
  private Optional<int[]> absoluteEncoder1TeethStages = Optional.empty();

  /**
   * Optional (driver, driven) stage pairs for encoder 2 gearing.
   */
  private Optional<int[]> absoluteEncoder2TeethStages = Optional.empty();

  /**
   * Whether encoder 1 output should be inverted.
   */
  private boolean encoder1Inverted = false;

  /**
   * Whether encoder 2 output should be inverted.
   */
  private boolean encoder2Inverted = false;

  /**
   * Creates a configuration with suppliers for both absolute encoders.
   *
   * @param absoluteEncoder1AngleSupplier supplier that returns the angle for encoder 1
   * @param absoluteEncoder2AngleSupplier supplier that returns the angle for encoder 2
   */
  public EasyCRTConfig(
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
   * mounted in reverse, and you cannot set inversion on-device, use {@link
   * #withAbsoluteEncoder1Inverted(boolean)} or {@link #withAbsoluteEncoder2Inverted(boolean)} to
   * flip the sign.
   *
   * @param encoder1RotPerMechanismRot rotations per mechanism rotation for encoder 1
   * @param encoder2RotPerMechanismRot rotations per mechanism rotation for encoder 2
   * @return this configuration for chaining
   */
  public EasyCRTConfig withEncoderRatios(
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
   * <p>If the encoder gears are driven by the turret gear itself, the common ratio is 1, and the
   * drive gear is the turret gear teeth.
   *
   * <p>Also seeds CRT gear recommendation inputs (stage1 gear teeth + stage2 ratio) so you can
   * call {@link #withCrtGearRecommendationConstraints(double, int, int, int)} afterward.
   *
   * @param commonRatio ratio between mechanism and drive gear
   * @param driveGearTeeth tooth count on the gear that drives both encoder pinions
   * @param absoluteEncoder1PinionTeeth tooth count on encoder 1 pinion
   * @param absoluteEncoder2PinionTeeth tooth count on encoder 2 pinion
   * @return this configuration for chaining
   */
  public EasyCRTConfig withCommonDriveGear(
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
   * <p>If your encoder supports on device zeroing/offsets, configure it there and keep these
   * offsets at zero to avoid double-offsetting.
   *
   * @param encoder1Offset offset to apply to encoder 1 reading
   * @param encoder2Offset offset to apply to encoder 2 reading
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoderOffsets(
      Angle encoder1Offset, Angle encoder2Offset) {
    this.absoluteEncoder1Offset = Objects.requireNonNull(encoder1Offset, "encoder1Offset");
    this.absoluteEncoder2Offset = Objects.requireNonNull(encoder2Offset, "encoder2Offset");
    return this;
  }

  /**
   * Sets the allowed mechanism angle range (in rotations).
   *
   * @param minAngle minimum allowed mechanism angle
   * @param maxAngle maximum allowed mechanism angle
   * @return this configuration for chaining
   */
  public EasyCRTConfig withMechanismRange(Angle minAngle, Angle maxAngle) {
    this.minMechanismAngle = Objects.requireNonNull(minAngle, "minAngle");
    this.maxMechanismAngle = Objects.requireNonNull(maxAngle, "maxAngle");
    return this;
  }

  /**
   * Sets the match tolerance for encoder 2 modular error (in rotations).
   *
   * <p>Lower values are reasonable when backlash and sensor noise are low, but too small can yield
   * no solution; high backlash mechanisms may need larger values. When backlash is the main source
   * of error and not noise, the tolerance can often be close to the backlash.
   *
   * <p>Experimenting with always pushing the mechanism to one side of the backlash can allow a
   * reduced tolerance, but teams should validate this to ensure it will always solve with their
   * specified tolerance. Tune based on {@code getLastErrorRotations()}.
   *
   * <p>Example: mechanism backlash to the encoders is about 1 degree. commonRatio = 11,
   * driveGearTeeth = 50, and encoder2PinionTeeth = 30. Ratio = 18.3333. 1 degree of mechanism
   * backlash is about 0.0509 rotations of the encoder. A tolerance of 0.06 rotations corresponds
   * to about 1.18 degrees at the mechanism (0.06 / 18.33333 * 360), which is only slightly higher
   * than the backlash.
   *
   * @param tolerance allowable modular error for encoder 2
   * @return this configuration for chaining
   */
  public EasyCRTConfig withMatchTolerance(Angle tolerance) {
    this.matchTolerance = Objects.requireNonNull(tolerance, "tolerance");
    return this;
  }

  /**
   * If true, flips encoder 1 ratio sign (use when the sensor is mounted reversed).
   *
   * <p>If your encoder supports on-device direction/inversion, configure it there and keep this
   * false to avoid double-inversion.
   *
   * @param inverted whether encoder 1 should be inverted
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder1Inverted(boolean inverted) {
    this.encoder1Inverted = inverted;
    return this;
  }

  /**
   * If true, flips encoder 2 ratio sign (use when the sensor is mounted reversed).
   *
   * <p>If your encoder supports on-device direction/inversion, configure it there and keep this
   * false to avoid double-inversion.
   *
   * @param inverted whether encoder 2 should be inverted
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder2Inverted(boolean inverted) {
    this.encoder2Inverted = inverted;
    return this;
  }

  /**
   * Sets inversion for both encoders in one call.
   *
   * @param encoder1Inverted whether encoder 1 should be inverted
   * @param encoder2Inverted whether encoder 2 should be inverted
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoderInversions(
      boolean encoder1Inverted, boolean encoder2Inverted) {
    this.encoder1Inverted = encoder1Inverted;
    this.encoder2Inverted = encoder2Inverted;
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
   *
   * @param stage1GearTeeth tooth count on the gear that drives both encoders
   * @param stage2Ratio common ratio between mechanism and drive gear
   * @return this configuration for chaining
   */
  public EasyCRTConfig withCrtGearRecommendationInputs(
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
   *
   * @param coverageMargin additional coverage margin multiplier
   * @param minTeeth minimum tooth count to search
   * @param maxTeeth maximum tooth count to search
   * @param maxIterationsLimit maximum iterations per gear to consider valid
   * @return this configuration for chaining
   */
  public EasyCRTConfig withCrtGearRecommendationConstraints(
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
   *
   * @param teethChain ordered teeth counts from mechanism gear to encoder 1 pinion
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder1Gearing(int... teethChain) {
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
   *
   * @param teethChain ordered teeth counts from mechanism gear to encoder 2 pinion
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder2Gearing(int... teethChain) {
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
   *
   * @param driverDrivenPairs alternating driver and driven teeth counts for each stage
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder1GearingStages(int... driverDrivenPairs) {
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
   *
   * @param driverDrivenPairs alternating driver and driven teeth counts for each stage
   * @return this configuration for chaining
   */
  public EasyCRTConfig withAbsoluteEncoder2GearingStages(int... driverDrivenPairs) {
    this.absoluteEncoder2TeethStages =
        Optional.of(copyTeeth(driverDrivenPairs, "absoluteEncoder2TeethStages"));
    return this;
  }

  /**
   * Builds a MechanismGearing for encoder 1 from the configured chain or stages.
   *
   * @return gearing representation for encoder 1
   */
  public MechanismGearing getAbsoluteEncoder1Gearing() {
    return buildMechanismGearingForEncoder(1);
  }

  /**
   * Builds a MechanismGearing for encoder 2 from the configured chain or stages.
   *
   * @return gearing representation for encoder 2
   */
  public MechanismGearing getAbsoluteEncoder2Gearing() {
    return buildMechanismGearingForEncoder(2);
  }

  /**
   * Builds mechanism gearing based on configured chain or stage pairs.
   *
   * @param encoderIndex encoder index (1 or 2) to build gearing for
   * @return gearing representation based on provided configuration
   */
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

  // --- Getters used by EasyCRT ---

  /**
   * Returns the current angle for absolute encoder 1.
   *
   * @return angle for encoder 1, or NaN rotation if supplier returns null
   */
  public Angle getAbsoluteEncoder1Angle() {
    Angle value = absoluteEncoder1AngleSupplier.get();
    return value != null ? value : Rotations.of(Double.NaN);
  }

  /**
   * Returns the current angle for absolute encoder 2.
   *
   * @return angle for encoder 2, or NaN rotation if supplier returns null
   */
  public Angle getAbsoluteEncoder2Angle() {
    Angle value = absoluteEncoder2AngleSupplier.get();
    return value != null ? value : Rotations.of(Double.NaN);
  }

  /**
   * Returns the configured offset for encoder 1 in rotations.
   *
   * @return offset applied to encoder 1
   */
  public Angle getAbsoluteEncoder1Offset() {
    return absoluteEncoder1Offset;
  }

  /**
   * Returns the configured offset for encoder 2 in rotations.
   *
   * @return offset applied to encoder 2
   */
  public Angle getAbsoluteEncoder2Offset() {
    return absoluteEncoder2Offset;
  }

  /**
   * Returns the minimum allowed mechanism rotations.
   *
   * @return minimum rotations for the mechanism
   */
  public Angle getMinMechanismAngle() {
    return minMechanismAngle;
  }

  /**
   * Returns the maximum allowed mechanism rotations.
   *
   * @return maximum rotations for the mechanism
   */
  public Angle getMaxMechanismAngle() {
    return maxMechanismAngle;
  }

  /**
   * Allowed mechanism travel in rotations (max - min).
   *
   * @return range of allowed mechanism motion
   */
  public Angle getMechanismRange() {
    return maxMechanismAngle.minus(minMechanismAngle);
  }

  /**
   * Returns the maximum allowed modular error between predicted and measured encoder 2.
   *
   * @return tolerance in rotations
   */
  public Angle getMatchTolerance() {
    return matchTolerance;
  }

  /**
   * Returns encoder 1 rotations per mechanism rotation (includes inversion if enabled).
   *
   * @return ratio for encoder 1
   */
  public double getEncoder1RotationsPerMechanismRotation() {
    double ratio = getOrComputeRatio(1);
    return encoder1Inverted ? -ratio : ratio;
  }

  /**
   * Returns encoder 2 rotations per mechanism rotation (includes inversion if enabled).
   *
   * @return ratio for encoder 2
   */
  public double getEncoder2RotationsPerMechanismRotation() {
    double ratio = getOrComputeRatio(2);
    return encoder2Inverted ? -ratio : ratio;
  }

  /**
   * Returns configured ratio or computes it from gearing for the specified encoder.
   *
   * @param encoderIndex encoder index (1 or 2)
   * @return rotations per mechanism rotation for the given encoder
   */
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
   *
   * @param teethChain ordered tooth counts from mechanism to encoder
   * @return encoder rotations per mechanism rotation
   */
  public static double ratioFromChain(int... teethChain) {
    String[] stages = buildStagesFromChain(teethChain);
    return new MechanismGearing(GearBox.fromStages(stages)).getMechanismToRotorRatio();
  }

  /**
   * Computes ratio from explicit (driver, driven) stage pairs.
   *
   * <p>Returns encoder rotations per mechanism rotation.
   *
   * @param driverDrivenPairs alternating driver and driven teeth counts for each stage
   * @return encoder rotations per mechanism rotation
   */
  public static double ratioFromDriverDrivenPairs(int... driverDrivenPairs) {
    String[] stages = buildStagesFromDriverDrivenPairs(driverDrivenPairs);
    return new MechanismGearing(GearBox.fromStages(stages)).getMechanismToRotorRatio();
  }

  /**
   * Computes ratio from a shared drive stage: commonRatio * (driveGear / encoderGear).
   *
   * <p>Returns encoder rotations per mechanism rotation.
   *
   * @param commonRatio ratio between mechanism and drive gear
   * @param driveGearTeeth tooth count on the gear that drives encoder pinions
   * @param encoderTeeth tooth count on the encoder pinion
   * @return encoder rotations per mechanism rotation
   */
  public static double ratioFromCommonDrive(
      double commonRatio, int driveGearTeeth, int encoderTeeth) {
    requireNonZeroFinite(commonRatio, "commonRatio");
    requirePositiveTeeth(driveGearTeeth, "driveGearTeeth");
    requirePositiveTeeth(encoderTeeth, "encoderTeeth");
    return commonRatio * (((double) driveGearTeeth) / encoderTeeth);
  }

  /**
   * Returns unique coverage in mechanism angle, if prime teeth + common scale are set.
   *
   * @return optional coverage range for the mechanism
   */
  public Optional<Angle> getUniqueCoverage() {
    if (encoder1PrimeTeeth.isEmpty() || encoder2PrimeTeeth.isEmpty() || commonScaleK.isEmpty()) {
      return Optional.empty();
    }
    double k = commonScaleK.get();
    if (!Double.isFinite(k) || Math.abs(k) < 1e-12) {
      return Optional.empty();
    }
    int l = lcm(encoder1PrimeTeeth.get(), encoder2PrimeTeeth.get());
    return Optional.of(Rotations.of(l / k));
  }

  /**
   * Returns true when the unique coverage meets or exceeds the configured maximum mechanism range.
   *
   * <p>Returns false if coverage inputs are missing or non-finite.
   *
   * @return whether coverage satisfies the configured mechanism travel
   */
  public boolean coverageSatisfiesRange() {
    Optional<Angle> coverageOpt = getUniqueCoverage();
    if (coverageOpt.isEmpty()) {
      return false;
    }
    double coverageRot = coverageOpt.get().in(Rotations);
    double maxRot = maxMechanismAngle.in(Rotations);
    if (!Double.isFinite(coverageRot) || !Double.isFinite(maxRot) || maxRot <= 0.0) {
      return false;
    }
    return coverageRot >= maxRot;
  }

  /**
   * Checks whether two integers are coprime.
   *
   * @param a first integer
   * @param b second integer
   * @return true when the numbers share no common factors beyond 1
   */
  public static boolean isCoprime(int a, int b) {
    return BigInteger.valueOf(a).gcd(BigInteger.valueOf(b)).equals(BigInteger.ONE);
  }

  /**
   * Returns a recommended CRT gear pair if gear-search inputs are configured.
   *
   * @return optional gear pair recommendation
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

    double maxMechanismRotations = maxMechanismAngle.in(Rotations);
    if (!Double.isFinite(maxMechanismRotations) || maxMechanismRotations <= 0.0) {
      return Optional.empty();
    }

    CrtGearPair pair =
        findSmallestCrtGearPair(
            gearSearchStage1GearTeeth.get(),
            gearSearchStage2Ratio.get(),
            maxMechanismAngle,
            gearSearchCoverageMargin.get(),
            gearSearchMinTeeth.get(),
            gearSearchMaxTeeth.get(),
            gearSearchMaxIterations.get());
    return Optional.ofNullable(pair);
  }

  /**
   * Finds the smallest gear pair that satisfies coverage and iteration limits.
   *
   * @param stage1GearTeeth tooth count on the gear that drives both encoders
   * @param stage2Ratio ratio between mechanism and drive gear
   * @param maxMechanismAngle maximum mechanism rotations to cover
   * @param coverageMargin multiplier to ensure enough unique coverage
   * @param minTeeth minimum teeth to consider
   * @param maxTeeth maximum teeth to consider
   * @param maxIterationsLimit maximum iterations allowed for a gear
   * @return gear pair that meets the criteria, or null if none found
   */
  public static CrtGearPair findSmallestCrtGearPair(
      int stage1GearTeeth,
      double stage2Ratio,
      Angle maxMechanismAngle,
      double coverageMargin,
      int minTeeth,
      int maxTeeth,
      int maxIterationsLimit) {
    double maxMechanismRotations = Objects.requireNonNull(maxMechanismAngle, "maxMechanismAngle").in(Rotations);
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
            theoreticalIterationsForGear(a, stage1GearTeeth, stage2Ratio, maxMechanismAngle);
        int iterationsB =
            theoreticalIterationsForGear(b, stage1GearTeeth, stage2Ratio, maxMechanismAngle);
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
                  assignedA,
                  assignedB,
                  lcm,
                  Rotations.of(coverageRot),
                  gcd(a, b),
                  assignedIterations);
        }
      }
    }

    return best;
  }

  /**
   * Computes the theoretical number of iterations for a gear based on ratio and travel.
   *
   * @param gearTeeth tooth count of the gear being evaluated
   * @param stage1GearTeeth tooth count for the gear driving the encoder
   * @param stage2Ratio ratio between mechanism and drive gear
   * @param maxMechanismAngle maximum mechanism travel to cover
   * @return theoretical iteration count
   */
  private static int theoreticalIterationsForGear(
      int gearTeeth, int stage1GearTeeth, double stage2Ratio, Angle maxMechanismAngle) {
    double ratioA = stage2Ratio * ((double) stage1GearTeeth / gearTeeth);
    return (int) Math.ceil(ratioA * maxMechanismAngle.in(Rotations)) + 3;
  }

  /**
   * Validates that a value is finite and non-zero.
   *
   * @param value numeric value to check
   * @param label label used in the error message
   */
  private static void requireNonZeroFinite(double value, String label) {
    if (!Double.isFinite(value) || Math.abs(value) < 1e-12) {
      throw new IllegalArgumentException(label + " must be finite and non-zero");
    }
  }

  /**
   * Validates that a tooth count is positive.
   *
   * @param value tooth count to validate
   * @param label label used in the error message
   */
  private static void requirePositiveTeeth(int value, String label) {
    if (value <= 0) {
      throw new IllegalArgumentException(label + " must be > 0");
    }
  }

  /**
   * Validates that a value is finite and positive.
   *
   * @param value numeric value to validate
   * @param label label used in the error message
   */
  private static void requirePositiveFinite(double value, String label) {
    if (!Double.isFinite(value) || value <= 0.0) {
      throw new IllegalArgumentException(label + " must be finite and > 0");
    }
  }

  /**
   * Validates that all provided tooth counts are positive.
   *
   * @param values array of tooth counts to validate
   * @param label label used in the error message
   */
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
   *
   * @param teethChain ordered tooth counts from mechanism to encoder
   * @return gearbox stage strings for the chain
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
   *
   * @param driverDrivenPairs alternating driver and driven teeth counts for each stage
   * @return gearbox stage strings for the provided stages
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

  /**
   * Copies the provided teeth array.
   *
   * @param teeth tooth counts to copy
   * @param label label used when validating input
   * @return copied teeth array
   */
  private static int[] copyTeeth(int[] teeth, String label) {
    Objects.requireNonNull(teeth, label);
    return Arrays.copyOf(teeth, teeth.length);
  }

  /**
   * Computes the greatest common divisor for two integers.
   *
   * @param a first integer
   * @param b second integer
   * @return greatest common divisor
   */
  private static int gcd(int a, int b) {
    int x = Math.abs(a), y = Math.abs(b);
    while (y != 0) {
      int t = y;
      y = x % y;
      x = t;
    }
    return x;
  }

  /**
   * Computes the least common multiple for two integers.
   *
   * @param a first integer
   * @param b second integer
   * @return least common multiple
   */
  private static int lcm(int a, int b) {
    return (a / gcd(a, b)) * b;
  }

  /**
   * Represents a candidate CRT gear pair.
   *
   * @param gearA first gear tooth count
   * @param gearB second gear tooth count
   * @param lcm least common multiple of the two gears
   * @param coverage coverage in mechanism rotations
   * @param gcd greatest common divisor of the two gears
   * @param theoreticalIterations predicted iterations required
   */
  public static record CrtGearPair(
      int gearA, int gearB, int lcm, Angle coverage, int gcd, int theoreticalIterations) {}
}
