package yams.units;

import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import java.math.BigInteger;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;
import java.util.stream.Collectors;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;

public class CRTAbsoluteEncoderConfig
{

  private Optional<int[]> absoluteEncoder1Teeth = Optional.empty();
  private Optional<int[]> absoluteEncoder2Teeth = Optional.empty();
  private Supplier<Angle> absoluteEncoder1AngleSupplier;
  private Supplier<Angle> absoluteEncoder2AngleSupplier;

  public CRTAbsoluteEncoderConfig(Supplier<Angle> absoluteEncoder1AngleSupplier,
                                  Supplier<Angle> absoluteEncoder2AngleSupplier)
  {
    this.absoluteEncoder1AngleSupplier = absoluteEncoder1AngleSupplier;
    this.absoluteEncoder2AngleSupplier = absoluteEncoder2AngleSupplier;
  }

  /**
   * Set the absolute encoder gearing from the turret to the absolute encoder.
   *
   * @param teeth Gear teeth count from the turret to the absolute encoder.
   * @return {@link CRTAbsoluteEncoderConfig} for chaining.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder1Gearing(int... teeth)
  {
    this.absoluteEncoder1Teeth = Optional.of(teeth);
    return this;
  }

  /**
   * Set the absolute encoder gearing from the turret to the absolute encoder.
   *
   * @param teeth Gear teeth count from the turret to the absolute encoder.
   * @return {@link CRTAbsoluteEncoderConfig} for chaining.
   */
  public CRTAbsoluteEncoderConfig withAbsoluteEncoder2Gearing(int... teeth)
  {
    this.absoluteEncoder2Teeth = Optional.of(teeth);
    return this;
  }

  /**
   * Checks if two integers are coprime, using the extended Euclidean algorithm.
   *
   * @param a Integer A
   * @param b Integer B
   * @return Coprime or not
   */
  public static boolean isCoprime(int a, int b)
  {
    return BigInteger.valueOf(a).gcd(BigInteger.valueOf(b)).equals(BigInteger.ONE);
  }

  /**
   * Get the absolute encoder prime gears. Verifies the absolute encoders are properly setup for the Chinese Remainder
   * Theorem.
   *
   * @return (AbsoluteEncoder1PrimeGear, AbsoluteEncoder2PrimeGear)
   */
  public Pair<Integer, Integer> getAbsoluteEncoderPrimeGears()
  {
    if (absoluteEncoder1Teeth.isEmpty() || absoluteEncoder2Teeth.isEmpty())
    {
      throw new IllegalStateException("Chinese Remainder Theorem Absolute encoder gearing not set.");
    }
    Set<Integer> absoluteEncoder1DifferentGears = Arrays.stream(absoluteEncoder1Teeth.get()).boxed()
                                                        .collect(Collectors.toSet());
    Set<Integer> absoluteEncoder2DifferentGears = Arrays.stream(absoluteEncoder2Teeth.get()).boxed()
                                                        .collect(Collectors.toSet());
    // Remove all common gear teeth between the 2 absolute encoders.
    absoluteEncoder1DifferentGears.removeAll(Arrays.stream(absoluteEncoder2Teeth.get()).boxed()
                                                   .collect(Collectors.toSet()));
    absoluteEncoder2DifferentGears.removeAll(Arrays.stream(absoluteEncoder1Teeth.get()).boxed()
                                                   .collect(Collectors.toSet()));
    if (absoluteEncoder1DifferentGears.size() != 1)
    {
      throw new IllegalStateException(
          "Chinese Remainder Theorem Absolute encoder 1 gearing contains more than one different gear.");
    }
    if (absoluteEncoder2DifferentGears.size() != 1)
    {
      throw new IllegalStateException(
          "Chinese Remainder Theorem Absolute encoder 2 gearing contains more than one different gear.");
    }

    var absoluteEncoder1PrimeGear = (Integer) absoluteEncoder1DifferentGears.toArray()[0];
    var absoluteEncoder2PrimeGear = (Integer) absoluteEncoder2DifferentGears.toArray()[0];
    if (!isCoprime(absoluteEncoder1PrimeGear, absoluteEncoder2PrimeGear))
    {
      throw new IllegalStateException(
          "Chinese Remainder Theorem Absolute encoder prime gears (" + absoluteEncoder1PrimeGear + ", " +
          absoluteEncoder2PrimeGear + ") are not coprime.");
    }
    return Pair.of(absoluteEncoder1PrimeGear, absoluteEncoder2PrimeGear);
  }

  /**
   * Get the absolute encoder gearing from the turret to the absolute encoder.
   *
   * @param teeth Teeth, starting from the mechanism to the absolute encoder.
   * @return Array of stages.
   */
  private String[] getAbsoluteEncoderStages(int[] teeth)
  {
    // TODO: Have someone check if this is right.
    ArrayList<String> gearings = new ArrayList<>();
    for (int i = 1; i < teeth.length; i++)
    {
      gearings.add(teeth[i - 1] + ":" + teeth[i]);
    }
    return gearings.toArray(String[]::new);
  }

  /**
   * Get the absolute encoder 1 gearing from the turret to the absolute encoder.
   *
   * @return Absolute encoder 1 {@link MechanismGearing}.
   */
  public MechanismGearing getAbsoluteEncoder1Gearing()
  {
    var teeth    = absoluteEncoder1Teeth.orElseThrow();
    var gearings = getAbsoluteEncoderStages(teeth);
//    System.out.println("Absolute Encoder 1 Gearing:");
//    System.out.println(Arrays.toString(gearings));
    return new MechanismGearing(GearBox.fromStages(gearings));
  }

  /**
   * Get the absolute encoder gearing from the turret to the absolute encoder.
   *
   * @return Absolute encoder 2 {@link MechanismGearing}.
   */
  public MechanismGearing getAbsoluteEncoder2Gearing()
  {

    var teeth    = absoluteEncoder2Teeth.orElseThrow();
    var gearings = getAbsoluteEncoderStages(teeth);
//    System.out.println("Absolute Encoder 2 Gearing:");
//    System.out.println(Arrays.toString(gearings));
    return new MechanismGearing(GearBox.fromStages(gearings));
  }

  /**
   * Get the common gearing between the two absolute encoders.
   *
   * @return Common {@link MechanismGearing}.
   */
  public MechanismGearing getCommonGearing()
  {
    var teeth1      = absoluteEncoder1Teeth.orElseThrow();
    var teeth2      = absoluteEncoder2Teeth.orElseThrow();
    var commonGears = Arrays.stream(teeth1).filter(t -> Arrays.stream(teeth2).anyMatch(t2 -> t == t2)).toArray();
    var gearings    = getAbsoluteEncoderStages(commonGears);
    System.out.println("CRT Common Gearing:");
    System.out.println(Arrays.toString(gearings));
    return gearings.length > 2 ? new MechanismGearing(GearBox.fromStages(gearings)) : new MechanismGearing(1.0);
  }

  /**
   * Get the current angle of the absolute encoders.
   *
   * @return Current angle of the absolute encoders.
   */
  public Angle getAbsoluteEncoder1Angle()
  {
    return absoluteEncoder1AngleSupplier.get();
  }

  /**
   * Get the current angle of the absolute encoders.
   *
   * @return Current angle of the absolute encoders.
   */
  public Angle getAbsoluteEncoder2Angle()
  {
    return absoluteEncoder2AngleSupplier.get();
  }

}
