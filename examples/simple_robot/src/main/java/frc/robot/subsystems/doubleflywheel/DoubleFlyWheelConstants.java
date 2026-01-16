package frc.robot.subsystems.doubleflywheel;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import java.util.List;

public class DoubleFlyWheelConstants
{

  /**
   * Utility Class for storing a measurement of a double flywheel that is known to be correct.
   *
   * @param distance      Distance from the center of the robot to the goal on a XY plane.
   * @param lowerVelocity Lower flywheel velocity to reach the goal.
   * @param upperVelocity Upper flywheel velocity to reach the goal.
   */
  private record DoubleFlyWheelMeasurement(Distance distance, AngularVelocity lowerVelocity,
                                           AngularVelocity upperVelocity)
  {

  }

  /**
   * Get the {@link InterpolatingDoubleTreeMap}s for the given {@link DoubleFlyWheelMeasurement}s.
   *
   * @param measurements {@link List} of {@link DoubleFlyWheelMeasurement}s to get the
   *                     {@link InterpolatingDoubleTreeMap}s for.
   * @return {@link Pair} of({@link InterpolatingDoubleTreeMap} lower, {@link InterpolatingDoubleTreeMap} upper).
   */
  private static Pair<InterpolatingDoubleTreeMap, InterpolatingDoubleTreeMap> getTreeMaps(
      List<DoubleFlyWheelMeasurement> measurements)
  {
    InterpolatingDoubleTreeMap lowerMap = new InterpolatingDoubleTreeMap();
    InterpolatingDoubleTreeMap upperMap = new InterpolatingDoubleTreeMap();

    for (var measurement : measurements)
    {
      lowerMap.put(measurement.distance().in(Meters), measurement.lowerVelocity().in(RPM));
      upperMap.put(measurement.distance().in(Meters), measurement.upperVelocity().in(RPM));
    }
    return Pair.of(lowerMap, upperMap);
  }


  /**
   * Pair of (lowerMap, upperMap)
   */
  public static Pair<InterpolatingDoubleTreeMap, InterpolatingDoubleTreeMap> distanceToRPM = getTreeMaps(List.of(
      new DoubleFlyWheelMeasurement(Inches.of(12), // 12in from center of goal to center of robot
                                    RPM.of(3000), // 3000 RPM for lower flywheel
                                    RPM.of(3000)), // 3000 RPM for upper flywheel
      new DoubleFlyWheelMeasurement(Meters.of(5), RPM.of(10000), RPM.of(36000)),
      new DoubleFlyWheelMeasurement(Meters.of(10), RPM.of(10000), RPM.of(36000))));
}