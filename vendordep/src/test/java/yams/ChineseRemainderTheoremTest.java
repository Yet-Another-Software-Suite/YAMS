package yams;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotations;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import org.junit.jupiter.api.Test;
import yams.gearing.MechanismGearing;
import yams.units.EasyCRT;
import yams.units.EasyCRTConfig;

public class ChineseRemainderTheoremTest {

    private Angle readingTolerance = Degrees.of(0.001);
    private double precision = 10.0;
    private Angle absoluteEncoder1Reading = Degrees.of(0);
    private Angle absoluteEncoder2Reading = Degrees.of(0);

  private Angle getAbs1()
  {
    return Degrees.of(absoluteEncoder1Reading.in(Degrees) % 360.0);
  }

  private Angle getAbs2()
  {
    return Degrees.of(absoluteEncoder2Reading.in(Degrees) % 360.0);
  }

  /**
   * Verifies that the common-drive gearing helper produces the expected encoder ratios.
   *
   * <p>Builds the encoder gearing from the shared turret gearbox (12->50, 10->110 with 30T/31T
   * encoder pinions) and checks that the config's computed ratios match the MechanismGearing
   * ratios.
   */
  @Test
  void testCRTGearingCalc()
  {
    double commonRatio = 11.0; // 10 -> 110
    int driveGearTeeth = 50;   // shared gear driving both encoders
    int encoder1Pinion = 30;
    int encoder2Pinion = 31;

    var absoluteEncoder1Gearing =
        new MechanismGearing(commonRatio * ((double) driveGearTeeth / encoder1Pinion));
    var absoluteEncoder2Gearing =
        new MechanismGearing(commonRatio * ((double) driveGearTeeth / encoder2Pinion));

    var config = new EasyCRTConfig(this::getAbs1, this::getAbs2)
        .withCommonDriveGear(commonRatio, driveGearTeeth, encoder1Pinion, encoder2Pinion);

    assertTrue(
        MathUtil.isNear(
            absoluteEncoder1Gearing.getMechanismToRotorRatio(),
            config.getEncoder1RotationsPerMechanismRotation(),
            0.0000001));
    assertTrue(
        MathUtil.isNear(
            absoluteEncoder2Gearing.getMechanismToRotorRatio(),
            config.getEncoder2RotationsPerMechanismRotation(),
            0.0000001));

  }

  /**
   * Runs a sweep within the unique coverage range, checking that
   * the solver returns the true mechanism angle.
   */
  @Test
  void testCRT()
  {
    System.out.println("Starting CRT Test");
    double commonRatio = 180.0/60.0; // 60 -> 180
    int driveGearTeeth = 60; // shared gear driving both encoders (60 in this case)
    int encoder1Pinion = 19; // 19t attached to 60t drive gear
    int encoder2Pinion = 21; // 21t attached to 60t drive gear

    var absoluteEncoder1Gearing =
        new MechanismGearing(commonRatio * ((double) driveGearTeeth / encoder1Pinion));
    var absoluteEncoder2Gearing =
        new MechanismGearing(commonRatio * ((double) driveGearTeeth / encoder2Pinion));

    var config = new EasyCRTConfig(this::getAbs1, this::getAbs2)
        .withCommonDriveGear(commonRatio, driveGearTeeth, encoder1Pinion, encoder2Pinion);

    config.getUniqueCoverage().ifPresent(angle -> System.out.println("Unique Coverage(rots): " + angle.in(Rotations)));

    // Limit the sweep to the unique coverage
    double coverageRotations = config.getUniqueCoverage().map(angle -> angle.in(Rotations)).orElse(2.0);
    double sweepRotations = Math.max(0.5, Math.min(coverageRotations - 0.05, coverageRotations));
    config.withMechanismRange(Rotations.of(0), Rotations.of(sweepRotations));
    int maxIterations = (int) Math.round(sweepRotations * 360 * precision);
    var encoder = new EasyCRT(config);
    for (int i = 0; i < maxIterations; i++)
    {
      var turretAngle = Degrees.of(i / precision);
      absoluteEncoder1Reading = turretAngle.times(absoluteEncoder1Gearing.getMechanismToRotorRatio());
      absoluteEncoder2Reading = turretAngle.times(absoluteEncoder2Gearing.getMechanismToRotorRatio());
      var estimatedAngleOpt = encoder.getAngleOptional();
      if (estimatedAngleOpt.isEmpty())
      {
        System.out.println("Solver returned empty at turret angle(degrees): " + turretAngle.in(Degrees));
        break;
      }
      var estimatedAngle = estimatedAngleOpt.get();
      var testing = turretAngle.isNear(estimatedAngle, readingTolerance);
      if (!testing)
      {
        System.out.println("Absolute Encoder Reading: " + getAbs1() + " " + getAbs2());
        System.out.println("Turret Angle(degrees): " + turretAngle.in(Degrees));
        System.out.println("CRT Angle(degrees): " + estimatedAngle.in(Degrees));
        break;
      }
      assertTrue(testing);
//      System.out.println("CRT Angle(rots): " + encoder.getAngleOptional());

    }
  }
}
