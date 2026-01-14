package yams;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;

public class ChineseRemainderTheoremTest
{

  private Angle  readingTolerance        = Degrees.of(1);
  private int    primeGear1              = 13;
  private int    primeGear2              = 23;
  private double precision               = 10.0;
  private int    maximumTestDegrees      = (int) Rotations.of(520).in(Degrees);
  private Angle  absoluteEncoder1Reading = Degrees.of(0);
  private Angle  absoluteEncoder2Reading = Degrees.of(0);

  private Angle getAbs1()
  {
    return absoluteEncoder1Reading;
  }

  private Angle getAbs2()
  {
    return absoluteEncoder2Reading;
  }

  @Test
  void testCRTGearingCalc()
  {
    var absoluteEncoder1Gearing = new MechanismGearing(GearBox.fromTeeth(primeGear1, 32, 20, 200));
    var absoluteEncoder2Gearing = new MechanismGearing(GearBox.fromTeeth(primeGear2, 32, 20, 200));

    var config = new CRTAbsoluteEncoderConfig(this::getAbs1, this::getAbs2)
        .withAbsoluteEncoder1Gearing(200, 20, 32, primeGear1)
        .withAbsoluteEncoder2Gearing(200, 20, 32, primeGear2);

    System.out.println(
        "Absolute Encoder 1 Mech to Rotor Ratio: \n" + config.getAbsoluteEncoder1Gearing().getMechanismToRotorRatio() + "==" +
        absoluteEncoder1Gearing.getMechanismToRotorRatio());

    System.out.println(
        "Absolute Encoder 2 Mech to Rotor Ratio: \n" + config.getAbsoluteEncoder2Gearing().getMechanismToRotorRatio() + "==" +
        absoluteEncoder2Gearing.getMechanismToRotorRatio());

    assertTrue(MathUtil.isNear(absoluteEncoder1Gearing.getMechanismToRotorRatio(),
                               config.getAbsoluteEncoder1Gearing().getMechanismToRotorRatio(), 0.0000001));
    assertTrue(MathUtil.isNear(absoluteEncoder2Gearing.getMechanismToRotorRatio(),
                               config.getAbsoluteEncoder2Gearing().getMechanismToRotorRatio(), 0.0000001));

  }

  @Test
  void testCRT()
  {

    var absoluteEncoder1Gearing = new MechanismGearing(GearBox.fromTeeth(primeGear1, 32, 20, 200));
    var absoluteEncoder2Gearing = new MechanismGearing(GearBox.fromTeeth(primeGear2, 32, 20, 200));

    var config = new CRTAbsoluteEncoderConfig(this::getAbs1, this::getAbs2)
        .withAbsoluteEncoder1Gearing(200, 20, 32, primeGear1)
        .withAbsoluteEncoder2Gearing(200, 20, 32, primeGear2);
    var encoder = new CRTAbsoluteEncoder(config);
    for (int i = 0; i < maximumTestDegrees * precision; i++)
    {
      var turretAngle = Degrees.of(i / precision);
      absoluteEncoder1Reading = turretAngle.times(absoluteEncoder1Gearing.getMechanismToRotorRatio());
      absoluteEncoder2Reading = turretAngle.times(absoluteEncoder2Gearing.getMechanismToRotorRatio());
      var testing = turretAngle.isNear(encoder.getAngle(), readingTolerance);
      if (!testing)
      {
        System.out.println("Absolute Encoder Reading: " + absoluteEncoder1Reading + " " + absoluteEncoder2Reading);
        System.out.println("Turret Angle(degrees): " + turretAngle.in(Degrees));
        System.out.println("CRT Angle(degrees): " + encoder.getAngle().in(Degrees));
        break;
      }
//      assertTrue(testing);
//      System.out.println("CRT Angle(rots): " + encoder.getAngle().in(Rotations));

    }

  }
}
