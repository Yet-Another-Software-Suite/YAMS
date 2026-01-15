package yams.units;

import static edu.wpi.first.units.Units.Degrees;
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

  static class Bezout {
    final double u, v;
    final int gcd;
    Bezout(double u, double v, int gcd) {
      this.u = u;
      this.v = v;
      this.gcd = gcd;
    }
  }

  static Bezout bezout(int a, int b) {
    int old_r = a, r = b;
    int old_s = 1, s = 0;
    int old_t = 0, t = 1;

    while (r != 0) {
      int q = old_r / r;
      int tmp;

      tmp = old_r; old_r = r; r = tmp - q * r;
      tmp = old_s; old_s = s; s = tmp - q * s;
      tmp = old_t; old_t = t; t = tmp - q * t;
    }

    return new Bezout(old_s, old_t, old_r);
  }

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
   */
  public Angle getAngle()
  {
    // This is still wrong, i have no idea why....
    double posA = config.getAbsoluteEncoder1Angle().in(Rotations);
    double posB = config.getAbsoluteEncoder2Angle().in(Rotations);

    double a = posA * m1;
    double b = posB * m2;

    Bezout bz = bezout(m1, m2);

    double m = (double) m1 * m2 / bz.gcd;

    double x =
        (a * bz.v * m2 +
         b * bz.u * m1) / bz.gcd;
    var turretRotations = Rotations.of(x * commonGearing.getRotorToMechanismRatio());
    return turretRotations;
  }

}
