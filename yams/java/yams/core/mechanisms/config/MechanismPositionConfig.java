// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Optional;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.core.mechanisms.positional.SmartPositionalMechanism;

/**
 * A configuration class for specifying the position and visualization properties of a mechanism
 * relative to a robot in a 3D coordinate system. This class allows setting and retrieving details
 * such as the mechanism's position relative to the robot, the robot's maximum dimensions, and the
 * plane in which the mechanism operates.
 *
 * <p>{@code MechanismPositionConfig} is used by {@link SmartPositionalMechanism} to drive the
 * built-in {@code Mechanism2d} widget in Glass/Shuffleboard. It captures three pieces of
 * information:
 *
 * <ul>
 * <li><b>Robot-relative position</b> a {@link org.wpilib.math.geometry.Translation3d}
 * that places the mechanism's pivot/root inside the robot frame. When set, the Mechanism2d
 * window is automatically offset so the mechanism appears in the correct position relative
 * to the robot outline.</li>
 * <li><b>Robot bounding box</b> optional maximum length and height used to size the
 * Mechanism2d canvas. When omitted, the canvas defaults to twice the mechanism's
 * reported length.</li>
 * <li><b>Movement plane</b> whether the mechanism moves in the XZ (default), YZ, or XY
 * plane of the robot coordinate system. This determines which Translation3d axes are
 * mapped to the 2D canvas X and Y axes.</li>
 * </ul>
 *
 * <p>It also places the mechanism in 3D, for each mechanism's {@code getPose3d()}: where it is mounted
 * on the robot, and how it is turned there, optionally on another mechanism, such as an arm on an
 * elevator's carriage.
 *
 * <h2>Example</h2>
 * <pre>{@code
 * import static org.wpilib.units.Units.Meters;
 * import org.wpilib.math.geometry.Translation3d;
 * import yams.core.mechanisms.config.MechanismPositionConfig;
 *
 * MechanismPositionConfig posConfig = new MechanismPositionConfig()
 *     .withRelativePosition(new Translation3d(0.2, 0.0, 0.3)) // 20 cm forward, 30 cm up
 *     .withMaxRobotLength(Meters.of(0.85))
 *     .withMaxRobotHeight(Meters.of(1.20))
 *     .withMovementPlane(MechanismPositionConfig.Plane.XZ);
 * }</pre>
 */
public class MechanismPositionConfig {
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
  protected Plane plane = Plane.XZ;
  /**
   * How the mechanism is turned where it is mounted.
   */
  protected Rotation3d mountRotation = Rotation3d.ZERO;
  /**
   * The pose the mechanism is mounted on, such as another mechanism's, or the robot's origin if empty.
   */
  protected Optional<Supplier<Pose3d>> parent = Optional.empty();

  /** Create a {@link MechanismPositionConfig} with no relative position or robot size limits. */
  public MechanismPositionConfig() {}

  /**
   * Set the position of the {@link SmartPositionalMechanism} relative to the robot.
   *
   * @param robotToMechanism {@link Pose3d} of the {@link SmartPositionalMechanism} relative to the
   *                         robot.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withRelativePosition(Translation3d robotToMechanism) {
    this.robotToMechanism = Optional.ofNullable(robotToMechanism);
    return this;
  }

  /**
   * Set where the mechanism is mounted, relative to the robot, or to its parent, and how it is turned
   * there. The mechanism's pose is worked out from here: an arm's or pivot's axis, an elevator's
   * carriage at zero height, or a flywheel's axle.
   *
   * @param robotToMechanism {@link Transform3d} from the robot, or from its parent, to the mechanism.
   * @return The {@link MechanismPositionConfig}, for easy chaining.
   */
  public MechanismPositionConfig withRelativePosition(Transform3d robotToMechanism) {
    this.robotToMechanism = Optional.of(robotToMechanism.getTranslation());
    this.mountRotation = robotToMechanism.getRotation();
    return this;
  }

  /**
   * Mount the mechanism on another, such as an arm on an elevator's carriage, so it moves with it. The
   * mechanism's relative position is then from the parent's pose, rather than from the robot.
   *
   * @param parentPose The pose the mechanism is mounted on, such as {@code elevator::getPose3d}.
   * @return The {@link MechanismPositionConfig}, for easy chaining.
   */
  public MechanismPositionConfig withParent(Supplier<Pose3d> parentPose) {
    this.parent = Optional.ofNullable(parentPose);
    return this;
  }

  /**
   * Set the length of the robot for visualization purposes.
   *
   * @param robotLength Length of the robot in meters.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMaxRobotLength(Distance robotLength) {
    this.maxRobotLength = Optional.ofNullable(robotLength);
    return this;
  }

  /**
   * Set the height of the robot for visualization purposes.
   *
   * @param robotHeight Height of the robot in meters.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMaxRobotHeight(Distance robotHeight) {
    this.maxRobotHeight = Optional.ofNullable(robotHeight);
    return this;
  }

  /**
   * Set the plane that the mechanism is on, used for position calculations.
   *
   * @param plane The plane that the mechanism is on. Default is X-Z plane.
   * @return The {@link SmartPositionalMechanism}, for easy chaining.
   */
  public MechanismPositionConfig withMovementPlane(Plane plane) {
    this.plane = plane;
    return this;
  }

  /**
   * Converts a given distance in the x-direction to a x-coordinate appropriate for visualizing on a
   * Mechanism2d.
   *
   * @param length the distance in the x-direction
   * @return the x-coordinate for visualizing on a Mechanism2d
   */
  public Distance getMechanismX(Distance length) {
    if (plane == Plane.YZ || plane == Plane.XY) {
      return robotToMechanism.map(rtm -> rtm.getMeasureY().plus(getWindowXDimension(length).div(2.0))).orElse(length);
    }
    return robotToMechanism.map(rtm -> rtm.getMeasureX().plus(getWindowXDimension(length).div(2.0))).orElse(length);
  }

  /**
   * Gets the distance in the y-direction appropriate for visualizing on a Mechanism2d, defaults to
   * the given distance.
   *
   * @param y the default distance in the y-direction
   * @return the y-coordinate for visualizing on a Mechanism2d
   */
  public Distance getMechanismY(Distance y) {
    return robotToMechanism.map(it -> it.getMeasureZ()).orElse(y);
  }

  /**
   * Returns the x dimension of the window in the Mechanism2d for visualization, either the max
   * robot length if set, or twice the given length.
   *
   * @param length the length of the mechanism
   * @return the x dimension of the window in the Mechanism2d
   */
  public Distance getWindowXDimension(Distance length) {
    return maxRobotLength.orElse(length.times(2));
  }

  /**
   * Returns the y dimension of the window in the Mechanism2d for visualization, either the max
   * robot height if set, or twice the given length.
   *
   * @param length the length of the mechanism
   * @return the y dimension of the window in the Mechanism2d
   */
  public Distance getWindowYDimension(Distance length) {
    return maxRobotHeight.orElse(length.times(2));
  }

  /**
   * Get the relative position of the mechanism to the robot.
   *
   * @return {@link Translation3d} representing the relative position. Defaults to a zero
   *         translation if not set.
   */
  public Optional<Translation3d> getRelativePosition() {
    return robotToMechanism;
  }

  /**
   * Get the plane that the mechanism is on.
   *
   * @return The {@link Plane} that the mechanism is on.
   */
  public Plane getMovementPlane() {
    return plane;
  }

  /**
   * Where the mechanism is mounted: its parent's pose, or the robot's origin, moved and turned by its
   * relative position.
   *
   * @return The {@link Pose3d} the mechanism is mounted at.
   */
  public Pose3d getMountPose() {
    Pose3d base = parent.map(Supplier::get).orElse(Pose3d.ZERO);
    return base.transformBy(new Transform3d(robotToMechanism.orElse(Translation3d.ZERO), mountRotation));
  }

  /**
   * The pose of a part that turns about the axis normal to the movement plane, such as an arm or a
   * pivot, at an angle. At 0 it points along +X in the XZ and XY planes, and along +Y in the YZ plane,
   * and a positive angle turns it toward +Z in the XZ and YZ planes, and toward +Y in the XY plane.
   *
   * @param angle The angle it is turned to.
   * @return Its {@link Pose3d}: where it is mounted, turned by the angle.
   */
  public Pose3d getRotatedPose(Angle angle) {
    return getMountPose().transformBy(new Transform3d(Translation3d.ZERO, rotationInPlane(angle)));
  }

  /**
   * The pose of a part that slides along the movement plane, such as an elevator's carriage.
   *
   * @param distance How far it has slid from where it is mounted.
   * @param angle    The direction it slides in, in the movement plane: 0 along +X (+Y in the YZ plane),
   *                 90 degrees along +Z (+Y in the XY plane).
   * @return Its {@link Pose3d}: where it is mounted, moved by the distance, and not turned.
   */
  public Pose3d getTranslatedPose(Distance distance, Angle angle) {
    double meters = distance.in(Meters);
    double cos = Math.cos(angle.in(Radians)) * meters;
    double sin = Math.sin(angle.in(Radians)) * meters;
    Translation3d slide = switch (plane) {
      case XZ -> new Translation3d(cos, 0, sin);
      case YZ -> new Translation3d(0, cos, sin);
      case XY -> new Translation3d(cos, sin, 0);
    };
    return getMountPose().transformBy(new Transform3d(slide, Rotation3d.ZERO));
  }

  /**
   * The rotation of a part turned by an angle about the axis normal to the movement plane.
   *
   * @param angle The angle.
   * @return The {@link Rotation3d}.
   */
  public Rotation3d rotationInPlane(Angle angle) {
    double radians = angle.in(Radians);
    return switch (plane) {
      // +X turns toward +Z: a negative pitch about +Y.
      case XZ -> new Rotation3d(0, -radians, 0);
      // +Y turns toward +Z: a positive roll about +X.
      case YZ -> new Rotation3d(radians, 0, 0);
      // +X turns toward +Y: a positive yaw about +Z.
      case XY -> new Rotation3d(0, 0, radians);
    };
  }

  /**
   * The planes that the mechanism could be on, used for position calculations.
   */
  public enum Plane {
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
