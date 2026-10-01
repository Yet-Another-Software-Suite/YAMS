// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import first.robot.utils.AllianceFlipUtil;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisAccelerations;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.spline.PoseWithCurvature;
import org.wpilib.math.trajectory.DrivetrainSplineSample;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectory;
import org.wpilib.math.trajectory.DrivetrainSplineTrajectoryParameterizer;
import org.wpilib.math.trajectory.HolonomicSample;
import org.wpilib.math.trajectory.HolonomicTrajectory;
import org.wpilib.math.trajectory.constraint.TrajectoryConstraint;
import org.wpilib.system.Filesystem;

/**
 * A path drawn in the PathPlanner GUI, read from {@code deploy/pathplanner/paths/<name>.path}.
 *
 * <p>PathPlannerLib's WPILib 2027 build is made for commands v2, which cannot be used alongside
 * commands v3, so this reads the same files and turns them into a WPILib {@link HolonomicTrajectory}. Each pair of waypoints is the cubic Bézier curve the
 * GUI draws. The curve is time parameterized by WPILib with the path's global constraints, its
 * constraint zones, and its start and end velocities. The robot heading moves from the ideal
 * starting rotation, through any rotation targets, to the goal end rotation, in step with the
 * robot's progress along the path. Event markers keep their names and commands, and become start
 * and end times along the trajectory.
 */
public final class PathPlannerPath
{

  /** Number of points sampled along each Bézier segment before time parameterization. */
  private static final int SAMPLES_PER_SEGMENT = 100;

  /**
   * An event marker on the path.
   *
   * @param name      Marker name from the GUI.
   * @param command   Command to run.
   * @param startTime Time along the trajectory the command starts, in seconds.
   * @param endTime   Time the command is canceled, for zoned markers. Point markers run until they
   *                  finish or the path ends.
   */
  public record EventMarker(String name, PathPlannerCommand command, double startTime, Optional<Double> endTime)
  {

  }

  private final String              name;
  private final HolonomicTrajectory trajectory;
  private final List<EventMarker>   eventMarkers;

  private PathPlannerPath(String name, HolonomicTrajectory trajectory, List<EventMarker> eventMarkers)
  {
    this.name = name;
    this.trajectory = trajectory;
    this.eventMarkers = eventMarkers;
  }

  /**
   * Load a path from the deploy directory.
   *
   * @param pathName Name of the path in the PathPlanner GUI, without the {@code .path} extension.
   * @return The path.
   * @throws IOException if the file cannot be read.
   */
  public static PathPlannerPath fromPathFile(String pathName) throws IOException
  {
    Path file = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("paths")
                          .resolve(pathName + ".path");
    return fromJson(pathName, PathPlannerJson.readObject(Files.readString(file)));
  }

  /** @return Name of the path. */
  public String getName()
  {
    return name;
  }

  /**
   * Get the trajectory, mirrored for the red alliance when needed.
   *
   * @param flip Mirror the trajectory with {@link AllianceFlipUtil}.
   * @return Trajectory with field relative velocities, blue alliance origin.
   */
  public HolonomicTrajectory getTrajectory(boolean flip)
  {
    if (!flip)
    {
      return trajectory;
    }
    List<HolonomicSample> flipped = new ArrayList<>();
    for (HolonomicSample sample : trajectory.getSamples())
    {
      // Rotational symmetry: positions mirror through the field center, and velocities and
      // accelerations reverse their X and Y components. Turning rates are unchanged.
      flipped.add(new HolonomicSample(sample.time,
                                      AllianceFlipUtil.flip(sample.pose),
                                      new ChassisVelocities(-sample.velocity.vx,
                                                            -sample.velocity.vy,
                                                            sample.velocity.omega),
                                      new ChassisAccelerations(-sample.acceleration.ax,
                                                               -sample.acceleration.ay,
                                                               sample.acceleration.alpha)));
    }
    return new HolonomicTrajectory(flipped);
  }

  /**
   * The pose the robot starts the path at: the first waypoint with the ideal starting rotation.
   *
   * @return Starting pose, blue alliance origin.
   */
  public Pose2d getStartingPose()
  {
    return trajectory.start().pose;
  }

  /** @return Event markers on the path, in order of their start times. */
  public List<EventMarker> getEventMarkers()
  {
    return eventMarkers;
  }

  @SuppressWarnings("unchecked")
  private static PathPlannerPath fromJson(String name, Map<String, Object> json)
  {
    List<Map<String, Object>> waypointsJson = (List<Map<String, Object>>) json.get("waypoints");
    List<Translation2d[]>     segments      = new ArrayList<>();
    for (int i = 0; i + 1 < waypointsJson.size(); i++)
    {
      segments.add(new Translation2d[]{
          translation(waypointsJson.get(i).get("anchor")),
          translation(waypointsJson.get(i).get("nextControl")),
          translation(waypointsJson.get(i + 1).get("prevControl")),
          translation(waypointsJson.get(i + 1).get("anchor"))});
    }

    // Sample the Bézier curves, remembering where along the path (the waypoint relative position
    // used by the GUI) each point is. The parameterizer keeps the same Pose2d objects, so the
    // positions can be looked up again from its samples.
    List<PoseWithCurvature>      points    = new ArrayList<>();
    IdentityHashMap<Pose2d, Double> positions = new IdentityHashMap<>();
    for (int segment = 0; segment < segments.size(); segment++)
    {
      for (int step = segment == 0 ? 0 : 1; step <= SAMPLES_PER_SEGMENT; step++)
      {
        double            t     = (double) step / SAMPLES_PER_SEGMENT;
        PoseWithCurvature point = bezierPoint(segments.get(segment), t);
        points.add(point);
        positions.put(point.pose, segment + t);
      }
    }

    Map<String, Object> globalConstraints = (Map<String, Object>) json.get("globalConstraints");
    double              maxVelocity       = number(globalConstraints.get("maxVelocity"));
    double              maxAcceleration   = number(globalConstraints.get("maxAcceleration"));

    List<TrajectoryConstraint> constraints = new ArrayList<>();
    for (Map<String, Object> zone : (List<Map<String, Object>>) json.get("constraintZones"))
    {
      Map<String, Object> zoneConstraints = (Map<String, Object>) zone.get("constraints");
      constraints.add(new ZoneConstraint(positions,
                                         number(zone.get("minWaypointRelativePos")),
                                         number(zone.get("maxWaypointRelativePos")),
                                         number(zoneConstraints.get("maxVelocity")),
                                         number(zoneConstraints.get("maxAcceleration"))));
    }

    Map<String, Object> idealStartingState = (Map<String, Object>) json.get("idealStartingState");
    Map<String, Object> goalEndState       = (Map<String, Object>) json.get("goalEndState");

    DrivetrainSplineTrajectory splineTrajectory = DrivetrainSplineTrajectoryParameterizer.parameterize(
        points,
        constraints,
        number(idealStartingState.get("velocity")),
        number(goalEndState.get("velocity")),
        maxVelocity,
        maxAcceleration,
        false);

    // Robot heading keyframes along the path: ideal start, rotation targets, goal end.
    List<double[]> rotationKeyframes = new ArrayList<>();
    rotationKeyframes.add(new double[]{0, number(idealStartingState.get("rotation"))});
    for (Map<String, Object> target : (List<Map<String, Object>>) json.get("rotationTargets"))
    {
      rotationKeyframes.add(new double[]{number(target.get("waypointRelativePos")),
                                         number(target.get("rotationDegrees"))});
    }
    rotationKeyframes.add(new double[]{segments.size(), number(goalEndState.get("rotation"))});
    rotationKeyframes.sort(Comparator.comparingDouble(keyframe -> keyframe[0]));

    List<DrivetrainSplineSample> splineSamples = splineTrajectory.getSamples();
    double[]                     sampleTimes   = new double[splineSamples.size()];
    double[]                     samplePositions = new double[splineSamples.size()];
    List<HolonomicSample>        samples       = new ArrayList<>();
    for (int i = 0; i < splineSamples.size(); i++)
    {
      DrivetrainSplineSample splineSample = splineSamples.get(i);
      sampleTimes[i] = splineSample.time;
      samplePositions[i] = positions.getOrDefault(splineSample.pose, 0.0);
      // The spline pose faces the direction of travel; a holonomic robot drives that way while
      // facing the interpolated heading.
      Rotation2d travelDirection = splineSample.pose.getRotation();
      double     speed           = splineSample.forwardVelocity();
      samples.add(new HolonomicSample(splineSample.time,
                                      new Pose2d(splineSample.pose.getTranslation(),
                                                 heading(rotationKeyframes, samplePositions[i])),
                                      new ChassisVelocities(speed * travelDirection.getCos(),
                                                            speed * travelDirection.getSin(),
                                                            0),
                                      new ChassisAccelerations()));
    }
    fillTurnRatesAndAccelerations(samples);

    List<EventMarker> eventMarkers = new ArrayList<>();
    for (Map<String, Object> marker : (List<Map<String, Object>>) json.get("eventMarkers"))
    {
      Object command = marker.get("command");
      if (command == null)
      {
        // Markers without a command only trigger PathPlannerLib EventTriggers, which this robot
        // does not use.
        continue;
      }
      Object endPosition = marker.get("endWaypointRelativePos");
      eventMarkers.add(new EventMarker(
          (String) marker.get("name"),
          PathPlannerCommand.fromJson((Map<String, Object>) command),
          timeAtPosition(sampleTimes, samplePositions, number(marker.get("waypointRelativePos"))),
          endPosition == null ? Optional.empty()
                              : Optional.of(timeAtPosition(sampleTimes, samplePositions, number(endPosition)))));
    }
    eventMarkers.sort(Comparator.comparingDouble(EventMarker::startTime));

    return new PathPlannerPath(name, new HolonomicTrajectory(samples), List.copyOf(eventMarkers));
  }

  /**
   * Turn rates come from the change in heading between samples, and accelerations from the change
   * in velocity, so that sampling between two samples moves smoothly from one to the next.
   */
  private static void fillTurnRatesAndAccelerations(List<HolonomicSample> samples)
  {
    for (int i = 0; i + 1 < samples.size(); i++)
    {
      HolonomicSample sample = samples.get(i);
      HolonomicSample next   = samples.get(i + 1);
      double          dt     = next.time - sample.time;
      sample.velocity.omega = dt > 0
                              ? next.pose.getRotation().minus(sample.pose.getRotation()).getRadians() / dt
                              : 0;
    }
    for (int i = 0; i + 1 < samples.size(); i++)
    {
      HolonomicSample sample = samples.get(i);
      HolonomicSample next   = samples.get(i + 1);
      double          dt     = next.time - sample.time;
      if (dt > 0)
      {
        sample.acceleration = new ChassisAccelerations((next.velocity.vx - sample.velocity.vx) / dt,
                                                       (next.velocity.vy - sample.velocity.vy) / dt,
                                                       (next.velocity.omega - sample.velocity.omega) / dt);
      }
    }
  }

  /** Heading at a waypoint relative position, interpolated between rotation keyframes. */
  private static Rotation2d heading(List<double[]> keyframes, double position)
  {
    for (int i = 0; i + 1 < keyframes.size(); i++)
    {
      double[] start = keyframes.get(i);
      double[] end   = keyframes.get(i + 1);
      if (position <= end[0])
      {
        double fraction = end[0] > start[0] ? (position - start[0]) / (end[0] - start[0]) : 1;
        return Rotation2d.fromDegrees(start[1]).interpolate(Rotation2d.fromDegrees(end[1]), fraction);
      }
    }
    return Rotation2d.fromDegrees(keyframes.get(keyframes.size() - 1)[1]);
  }

  /** First trajectory time at which the robot reaches a waypoint relative position. */
  private static double timeAtPosition(double[] times, double[] positions, double position)
  {
    for (int i = 0; i < times.length; i++)
    {
      if (positions[i] >= position)
      {
        return times[i];
      }
    }
    return times[times.length - 1];
  }

  /**
   * Point on a cubic Bézier segment. The pose faces the direction of travel, as WPILib's spline
   * points do.
   */
  private static PoseWithCurvature bezierPoint(Translation2d[] p, double t)
  {
    double u = 1 - t;
    double x = u * u * u * p[0].getX() + 3 * u * u * t * p[1].getX() + 3 * u * t * t * p[2].getX()
               + t * t * t * p[3].getX();
    double y = u * u * u * p[0].getY() + 3 * u * u * t * p[1].getY() + 3 * u * t * t * p[2].getY()
               + t * t * t * p[3].getY();
    double dx = 3 * u * u * (p[1].getX() - p[0].getX()) + 6 * u * t * (p[2].getX() - p[1].getX())
                + 3 * t * t * (p[3].getX() - p[2].getX());
    double dy = 3 * u * u * (p[1].getY() - p[0].getY()) + 6 * u * t * (p[2].getY() - p[1].getY())
                + 3 * t * t * (p[3].getY() - p[2].getY());
    double ddx = 6 * u * (p[2].getX() - 2 * p[1].getX() + p[0].getX())
                 + 6 * t * (p[3].getX() - 2 * p[2].getX() + p[1].getX());
    double ddy = 6 * u * (p[2].getY() - 2 * p[1].getY() + p[0].getY())
                 + 6 * t * (p[3].getY() - 2 * p[2].getY() + p[1].getY());
    double speedSquared = dx * dx + dy * dy;
    double curvature    = speedSquared > 1e-12 ? (dx * ddy - dy * ddx) / Math.pow(speedSquared, 1.5) : 0;
    return new PoseWithCurvature(new Pose2d(x, y, new Rotation2d(dx, dy)), curvature);
  }

  @SuppressWarnings("unchecked")
  private static Translation2d translation(Object json)
  {
    Map<String, Object> point = (Map<String, Object>) json;
    return new Translation2d(number(point.get("x")), number(point.get("y")));
  }

  private static double number(Object json)
  {
    return ((Number) json).doubleValue();
  }

  /** A PathPlanner constraint zone: lower velocity and acceleration limits over part of the path. */
  private record ZoneConstraint(IdentityHashMap<Pose2d, Double> positions, double minPosition, double maxPosition,
                                double maxVelocity, double maxAcceleration) implements TrajectoryConstraint
  {

    private boolean contains(Pose2d pose)
    {
      Double position = positions.get(pose);
      return position != null && position >= minPosition && position <= maxPosition;
    }

    @Override
    public double getMaxVelocity(Pose2d pose, double curvature, double velocity)
    {
      return contains(pose) ? maxVelocity : Double.POSITIVE_INFINITY;
    }

    @Override
    public MinMax getMinMaxAcceleration(Pose2d pose, double curvature, double speed)
    {
      return contains(pose) ? new MinMax(-maxAcceleration, maxAcceleration) : new MinMax();
    }
  }
}
