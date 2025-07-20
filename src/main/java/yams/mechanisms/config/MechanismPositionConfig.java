// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package yams.mechanisms.config;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Distance;
import java.util.Optional;
import yams.mechanisms.positional.SmartPositionalMechanism;

public class MechanismPositionConfig
{

  /**
   * The translation from the robot to the mechanism (Optional)
   */
  protected Optional<Translation3d> robotToMechanism = Optional.empty();
  /**
   * The length of the robot in meters.
   */
  protected Optional<Distance>      maxRobotLength = Optional.empty();
  /**
   * The height of the robot in meters.
   */
  protected Optional<Distance>      maxRobotHeight = Optional.empty();
  // TODO: Add soft limits display config.
  // TODO: Add hard limits display config.
  /**
   * The plane that the mechanism is on, used for position calculations.
   */
  protected Plane plane = Plane.XZ;

  /**
   * Set the position of the {@link SmartPositionalMechanism} relative to the robot.
   *
   * @param robotToMechanism {@link Pose3d} of the {@link SmartPositionalMechanism} relative to the robot.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withRelativePosition(Translation3d robotToMechanism)
  {
    this.robotToMechanism = Optional.ofNullable(robotToMechanism);
    return this;
  }

  /**
   * Set the length of the robot for visualization purposes.
   *
   * @param robotLength Length of the robot in meters.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMaxRobotLength(Distance robotLength)
  {
    this.maxRobotLength = Optional.ofNullable(robotLength);
    return this;
  }

  /**
   * Set the height of the robot for visualization purposes.
   *
   * @param robotHeight Height of the robot in meters.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMaxRobotHeight(Distance robotHeight)
  {
    this.maxRobotHeight = Optional.ofNullable(robotHeight);
    return this;
  }

  /**
   * Set the plane that the mechanism is on, used for position calculations.
   *
   * @param plane The plane that the mechanism is on. Default is X-Z plane.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMovementPlane(Plane plane)
  {
    this.plane = plane;
    return this;
  }

  /**
   * Converts a given distance in the x-direction to a x-coordinate appropriate for visualizing on a Mechanism2d.
   *
   * @param length the distance in the x-direction
   * @return the x-coordinate for visualizing on a Mechanism2d
   */
  public Distance getMechanismX(Distance length)
  {
    if (plane == Plane.YZ || plane == Plane.XY)
    {
      return robotToMechanism.map(rtm -> rtm.getMeasureY().plus(getWindowXDimension(length).div(2.0))).orElse(
          length);
    }
    return robotToMechanism.map(rtm -> rtm.getMeasureX().plus(getWindowXDimension(length).div(2.0))).orElse(
        length);
  }

  /**
   * Gets the distance in the y-direction appropriate for visualizing on a Mechanism2d, defaults to the given distance.
   *
   * @param y the default distance in the y-direction
   * @return the y-coordinate for visualizing on a Mechanism2d
   */
  public Distance getMechanismY(Distance y)
  {
    return robotToMechanism.map(it -> it.getMeasureZ()).orElse(y);
  }

  /**
   * Returns the x dimension of the window in the Mechanism2d for visualization, either the max robot length if set, or
   * twice the given length.
   *
   * @param length the length of the mechanism
   * @return the x dimension of the window in the Mechanism2d
   */
  public Distance getWindowXDimension(Distance length)
  {
    return maxRobotLength.orElse(length.times(2));
  }

  /**
   * Returns the y dimension of the window in the Mechanism2d for visualization, either the max robot height if set, or
   * twice the given length.
   *
   * @param length the length of the mechanism
   * @return the y dimension of the window in the Mechanism2d
   */
  public Distance getWindowYDimension(Distance length)
  {
    return maxRobotHeight.orElse(length.times(2));
  }

  /**
   * Get the relative position of the mechanism to the robot.
   *
   * @return {@link Translation3d} representing the relative position. Defaults to a zero translation if not set.
   */
  public Optional<Translation3d> getRelativePosition()
  {
    return robotToMechanism;
  }

  /**
   * Get the plane that the mechanism is on.
   *
   * @return The {@link Plane} that the mechanism is on.
   */
  public Plane getMovementPlane()
  {
    return plane;
  }

  /**
   * The planes that the mechanism could be on, used for position calculations.
   */
  public enum Plane
  {XZ, YZ, XY}
}
