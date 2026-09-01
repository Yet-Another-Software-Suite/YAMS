// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechanisms.config;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Distance;
import java.util.Optional;
import yams.mechanisms.positional.SmartPositionalMechanism;

/**
 * A configuration class for specifying the position and visualization properties of a mechanism relative to a robot in
 * a 3D coordinate system. This class allows setting and retrieving details such as the mechanism's position relative to
 * the robot, the robot's maximum dimensions, and the plane in which the mechanism operates.
 *
 * <p>{@code MechanismPositionConfig} is used by {@link SmartPositionalMechanism} to drive the
 * built-in {@code Mechanism2d} widget in Glass/Shuffleboard. It captures three pieces of
 * information:
 *
 * <ul>
 *   <li><b>Robot-relative position</b> — a {@link org.wpilib.math.geometry.Translation3d}
 *       that places the mechanism's pivot/root inside the robot frame. When set, the Mechanism2d
 *       window is automatically offset so the mechanism appears in the correct position relative
 *       to the robot outline.</li>
 *   <li><b>Robot bounding box</b> — optional maximum length and height used to size the
 *       Mechanism2d canvas. When omitted, the canvas defaults to twice the mechanism's
 *       reported length.</li>
 *   <li><b>Movement plane</b> — whether the mechanism moves in the XZ (default), YZ, or XY
 *       plane of the robot coordinate system. This determines which Translation3d axes are
 *       mapped to the 2D canvas X and Y axes.</li>
 * </ul>
 *
 * <h2>Example</h2>
 * <pre>{@code
 * import static org.wpilib.units.Units.Meters;
 * import org.wpilib.math.geometry.Translation3d;
 * import yams.mechanisms.config.MechanismPositionConfig;
 *
 * MechanismPositionConfig posConfig = new MechanismPositionConfig()
 *     .withRelativePosition(new Translation3d(0.2, 0.0, 0.3)) // 20 cm forward, 30 cm up
 *     .withMaxRobotLength(Meters.of(0.85))
 *     .withMaxRobotHeight(Meters.of(1.20))
 *     .withMovementPlane(MechanismPositionConfig.Plane.XZ);
 * }</pre>
 */
public class MechanismPositionConfig
{
  /**
   * The translation from the robot to the mechanism (Optional)
   */
  protected Optional<Translation3d> robotToMechanism = Optional.empty();
  /**
   * The length of the robot in meters.
   */
  protected Optional<Distance>      maxRobotLength   = Optional.empty();
  /**
   * The height of the robot in meters.
   */
  protected Optional<Distance>      maxRobotHeight   = Optional.empty();
  // TODO: Add soft limits display config.
  // TODO: Add hard limits display config.
  /**
   * The plane that the mechanism is on, used for position calculations.
   */
  protected Plane                   plane            = Plane.XZ;

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
  {
    /**
     * X-Z Plane
     */
    XZ,
    /**
     * Y-Z Plane
     */
    YZ,
    /**
     * X-Y Plane
     */
    XY
  }
}
