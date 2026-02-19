package frc.robot.utils.sotm;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import java.util.List;

public class InterpolatingTOFTreeMap
{

  public  Time                       latencyCompensation = Milliseconds.of(150);
  private Distance                   m_flywheelCircumference;
  private List<LinearVelocityVector> m_measurements;
  private InterpolatingDoubleTreeMap m_map               = new InterpolatingDoubleTreeMap();

  public InterpolatingTOFTreeMap(Distance flywheelCircumference)
  {
    m_flywheelCircumference = flywheelCircumference;
  }

  public InterpolatingTOFTreeMap(List<LinearVelocityVector> measurements, Distance flywheelCircumference)
  {
    m_flywheelCircumference = flywheelCircumference;
    this.m_measurements = measurements;
    for (var vector : measurements)
    {
      m_map.put(vector.target.getTranslation().getDistance(vector.position.getTranslation()),
                vector.flywheelVelocity.in(RotationsPerSecond));
    }
  }

  public LinearVelocityVector get(LinearVelocityVector input)
  {
    var fieldOrientChassisSpeed = input.velocity;
    // 1. Latency compensation
    var estimatedPose = input.estimatePose(latencyCompensation);
    // 2. Target vector
    var targetVector   = input.target.getTranslation().minus(estimatedPose);
    var targetDistance = targetVector.getNorm();
    // Calculate the ideal exit velocity magnitude (based on distance)
    targetVector = targetVector.div(targetVector.getNorm()).times(m_map.get(targetDistance));

    // 3. Shot vector
    var shotVector = targetVector.minus(input.getLinearVelocity());

    return new LinearVelocityVector(new Pose2d(estimatedPose, estimatedPose.getAngle()),
                                    input.target,
                                    new ChassisSpeeds(shotVector.getX(),
                                                      shotVector.getY(),
                                                      shotVector.getAngle().getRadians()),
                                    RotationsPerSecond.of(shotVector.getNorm()));
  }
}
