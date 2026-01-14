package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import java.math.BigInteger;
import yams.gearing.MechanismGearing;

public class CRTAbsoluteEncoder
{

  private final MechanismGearing commonGearing;

  /**
   * Absolute encoder 1 prime gear teeth.
   */
  private final int m1;
  /**
   * Absolute encoder 2 prime gear teeth.
   */
  private final int m2;

  /**
   * CRT Config
   */
  private final CRTAbsoluteEncoderConfig config;

  // Precomputed CRT constants
  /**
   * Combined modulus = primeGearTeeth1 * primeGearTeeth2
   */
  private final int M;
  /**
   * Combined modulus / absolute encoder 1 prime gear teeth
   */
  private final int M1;
  /**
   * Combined modulus / absolute encoder 2 prime gear teeth
   */
  private final int M2;
  /**
   * Inverse modulus of (combined modules / absolute encoder 1 prime gear teeth).
   */
  private final int invM1;
  /**
   * Inverse modulus of (combined modules / absolute encoder 2 prime gear teeth).
   */
  private final int invM2;

  /**
   * Invert the modulus operation mathematically.
   *
   * @param a   a
   * @param mod modulus
   * @return Inverse of a mod modulus.
   */
  private static int modInverse(int a, int mod)
  {
    return BigInteger.valueOf(a)
                     .modInverse(BigInteger.valueOf(mod))
                     .intValue();
  }

  /**
   * Construct the Chinese Remainder Theorem Absolute Encoder.
   *
   * @param cfg {@link CRTAbsoluteEncoderConfig} with the absolute encoders setup.
   */
  public CRTAbsoluteEncoder(CRTAbsoluteEncoderConfig cfg)
  {
    config = cfg;
    var primeGears = cfg.getAbsoluteEncoderPrimeGears();
    m1 = primeGears.getFirst();
    m2 = primeGears.getSecond();
    commonGearing = cfg.getCommonGearing();

    this.M = m1 * m2;

    this.M1 = M / m1;
    this.M2 = M / m2;

//    System.out.println("prime gear1: " + m1);
//    System.out.println("prime gear2: " + m2);
//
//    System.out.println("M1: " + M1);
//    System.out.println("M2: " + M2);
    this.invM1 = modInverse(M1, m1);
    this.invM2 = modInverse(M2, m2);
  }

  /**
   * Get the current angle of the mechanism with the Chinese Remainder Theorem.
   *
   * @return Current angle of the mechanism.
   * @implNote Returns the mechanism rotation in the range of (0, absoluteEncoder1PrimeGearTeeth *
   * absoluteEncoder2PrimeGearTeeth) * rotorToMechanismRatio.
   */
  public Angle getAngle()
  {
    double enc1 = config.getAbsoluteEncoder1Angle().in(Rotations);

// Integer tooth indices
    int r1 = Math.floorMod((int) Math.floor(enc1 * m1), m1);
    int r2 = Math.floorMod((int) Math.floor(config.getAbsoluteEncoder2Angle().in(Rotations) * m2), m2);

// CRT
    int primeGearIndex = Math.floorMod(
        r1 * M1 * invM1 +
        r2 * M2 * invM2,
        M);

// Fractional part *in prime-gear domain*
    double fractionalPrimeGear = (enc1 * m1 - Math.floor(enc1 * m1)) / m1;

// Combine
    double primeGearRotations = primeGearIndex + fractionalPrimeGear;

    return Rotations.of(primeGearRotations * commonGearing.getRotorToMechanismRatio());

  }

}
