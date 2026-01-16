package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.math.MathUtil;
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
    double commonGear = Math.round(config.getAbsoluteEncoder1Gearing().getMechanismToRotorRatio() * m1 /
                                   config.getCommonGearing().getMechanismToRotorRatio());
    double enc1       = config.getAbsoluteEncoder1Angle().in(Rotations) * m1 / commonGear;
    double enc2       = config.getAbsoluteEncoder2Angle().in(Rotations) * m2 / commonGear;
    double closest1   = enc1, closest2 = enc2;
    for (int i = 0; i < 100; i++)
    {
      if (MathUtil.isNear(enc1, enc2, 0.0000000000001)) {break;}
      if (enc1 < enc2) {enc1 += m1 / commonGear;} else if (enc2 < enc1) {enc2 += m2 / commonGear;}
      if (Math.abs(enc1 - enc2) < Math.abs(closest1 - closest2))
      {
        closest1 = enc1;
        closest2 = enc2;
      }
    }
    return Rotations.of((closest1 + closest2) / 2 / commonGearing.getMechanismToRotorRatio());
  }
}
