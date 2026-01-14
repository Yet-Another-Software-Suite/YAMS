package yams.units;

import static edu.wpi.first.units.Units.Rotations;

import edu.wpi.first.units.measure.Angle;
import java.math.BigInteger;
import yams.gearing.MechanismGearing;

public class CRTAbsoluteEncoder
{

  private final MechanismGearing absoluteEncoder1Gearing;
  private final MechanismGearing absoluteEncoder2Gearing;

  private final int absoluteEncoder1PrimeGear;
  private final int absoluteEncoder2PrimeGear;

  private final CRTAbsoluteEncoderConfig config;

  // Precomputed CRT constants
  private final int M;  // Combined modulus = primeGearTeeth1 * primeGearTeeth2
  private final int M1;
  private final int M2;
  private final int invM1;
  private final int invM2;

  private static int modInverse(int a, int mod)
  {
    return BigInteger.valueOf(a)
                     .modInverse(BigInteger.valueOf(mod))
                     .intValue();
  }

  public CRTAbsoluteEncoder(CRTAbsoluteEncoderConfig cfg)
  {
    config = cfg;
    var primeGears = cfg.getAbsoluteEncoderPrimeGears();
    absoluteEncoder1PrimeGear = primeGears.getFirst();
    absoluteEncoder2PrimeGear = primeGears.getSecond();
    absoluteEncoder1Gearing = cfg.getAbsoluteEncoder1Gearing();
    absoluteEncoder2Gearing = cfg.getAbsoluteEncoder2Gearing();

    this.M = absoluteEncoder1PrimeGear * absoluteEncoder1PrimeGear;

    this.M1 = M / absoluteEncoder1PrimeGear;
    this.M2 = M / absoluteEncoder2PrimeGear;

    this.invM1 = modInverse(M1, absoluteEncoder1PrimeGear);
    this.invM2 = modInverse(M2, absoluteEncoder2PrimeGear);
  }

  /**
   * Get the current angle of the mechanism with the Chinese Remainder Theorem.
   *
   * @return Current angle of the mechanism.
   * @implNote In the range of (0, absoluteEncoder1PrimeGearTeeth * absoluteEncoder2PrimeGearTeeth)
   */
  public Angle getAngle()
  {
    int r1 = Math.floorMod((int) Math.round(
        config.getAbsoluteEncoder1Angle().in(Rotations) * absoluteEncoder1PrimeGear), absoluteEncoder1PrimeGear);
    int r2 = Math.floorMod((int) Math.round(
        config.getAbsoluteEncoder2Angle().in(Rotations) * absoluteEncoder2PrimeGear), absoluteEncoder2PrimeGear);

    int x =
        (r1 * M1 * invM1 +
         r2 * M2 * invM2) % M;

    return Rotations.of(Math.floorMod(x, M));
  }

}
