// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.config;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import yams.core.exceptions.DoubleJointedArmConfigurationException;
import yams.core.mechanisms.positional.DoubleJointedArm;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Double jointed arm configuration: the lower (shoulder) and upper (elbow) arms, and the exclusion zones the arm must
 * never enter while it travels.
 *
 * <p>Both arm angles are measured from horizontal, so the upper arm's angle does not change when only the lower arm
 * moves. The arm's frame has its origin at the shoulder, with X away from the shoulder and Y up, the same as
 * {@link DoubleJointedArm#getPosition()} and {@link DoubleJointedArm#getAnglesForPosition(Translation2d, boolean)}.
 *
 * <p>When exclusion zones are configured, every move to an angle or position follows a path planned with A* in joint
 * space, so neither segment of the arm passes through a zone on the way.
 *
 * <h2>Configuration Example</h2>
 * <pre>{@code
 * DoubleJointedArmConfig config = new DoubleJointedArmConfig(lowerArmConfig, lowerSMC, upperArmConfig, upperSMC)
 *     // The robot's bumpers and frame, from the shoulder: 0.3 m behind it to 0.3 m in front, up to 0.2 m below it.
 *     .withExclusionZone(new Translation2d(-0.3, -0.5), new Translation2d(0.3, -0.2))
 *     // Keep 2 inches away from the zones.
 *     .withExclusionZoneClearance(Inches.of(2));
 *
 * DoubleJointedArm arm = new DoubleJointedArm(config);
 * }</pre>
 */
public class DoubleJointedArmConfig {
  /** Lower (shoulder) {@link ArmConfig}. */
  private final ArmConfig            lowerConfig;
  /** {@link SmartMotorController} driving the lower joint. */
  private final SmartMotorController lowerSMC;
  /** Upper (elbow) {@link ArmConfig}. */
  private final ArmConfig            upperConfig;
  /** {@link SmartMotorController} driving the upper joint. */
  private final SmartMotorController upperSMC;
  /** Exclusion zones, each the box between two opposite corners, in the arm's frame. */
  private final List<ExclusionZone>  exclusionZones = new ArrayList<>();
  /** Distance the arm keeps from every exclusion zone. */
  private Distance                   clearance      = Meters.of(0);
  /** Spacing of the joint space grid the path is planned on. */
  private Angle                      pathResolution = Degrees.of(2);
  /** How close each joint must get to a waypoint before moving on, or empty for half the resolution. */
  private Optional<Angle>            pathTolerance  = Optional.empty();

  /**
   * Double jointed arm configuration.
   *
   * @param lowerConfig Lower (shoulder) {@link ArmConfig}.
   * @param lowerSMC    {@link SmartMotorController} driving the lower joint.
   * @param upperConfig Upper (elbow) {@link ArmConfig}.
   * @param upperSMC    {@link SmartMotorController} driving the upper joint.
   */
  public DoubleJointedArmConfig(ArmConfig lowerConfig, SmartMotorController lowerSMC, ArmConfig upperConfig, SmartMotorController upperSMC) {
    this.lowerConfig = Objects.requireNonNull(lowerConfig, "lowerConfig cannot be null");
    this.lowerSMC = Objects.requireNonNull(lowerSMC, "lowerSMC cannot be null");
    this.upperConfig = Objects.requireNonNull(upperConfig, "upperConfig cannot be null");
    this.upperSMC = Objects.requireNonNull(upperSMC, "upperSMC cannot be null");
  }

  /**
   * Add an exclusion zone the arm must never enter: the box between two opposite corners, in the arm's frame. Either
   * pair of opposite corners works, in either order.
   *
   * @param corner     A corner of the box, in meters from the shoulder.
   * @param opposite   The opposite corner of the box, in meters from the shoulder.
   * @return {@link DoubleJointedArmConfig} for chaining.
   * @throws DoubleJointedArmConfigurationException if the box has no area.
   */
  public DoubleJointedArmConfig withExclusionZone(Translation2d corner, Translation2d opposite) {
    Objects.requireNonNull(corner, "corner cannot be null");
    Objects.requireNonNull(opposite, "opposite cannot be null");
    if (corner.getX() == opposite.getX() || corner.getY() == opposite.getY()) {
      throw new DoubleJointedArmConfigurationException("Exclusion zone " + corner + " to " + opposite + " has no area",
                                                       "Cannot plan around the exclusion zone.",
                                                       "withExclusionZone(Translation2d,Translation2d) with opposite corners of the box");
    }
    exclusionZones.add(new ExclusionZone(corner, opposite));
    return this;
  }

  /**
   * Set the distance the arm keeps from every exclusion zone, so following the path closely is enough to stay out of
   * them. Defaults to 0.
   *
   * @param clearance {@link Distance} to keep from the exclusion zones.
   * @return {@link DoubleJointedArmConfig} for chaining.
   * @throws DoubleJointedArmConfigurationException if the clearance is negative.
   */
  public DoubleJointedArmConfig withExclusionZoneClearance(Distance clearance) {
    Objects.requireNonNull(clearance, "clearance cannot be null");
    if (clearance.in(Meters) < 0) {
      throw new DoubleJointedArmConfigurationException("Exclusion zone clearance is negative", "Cannot plan around the exclusion zones.",
                                                       "withExclusionZoneClearance(Distance) with 0 or more");
    }
    this.clearance = clearance;
    return this;
  }

  /**
   * Set the spacing of the joint space grid paths around the exclusion zones are planned on. A finer grid finds paths
   * through tighter gaps but takes longer to plan. Defaults to 2 degrees.
   *
   * @param resolution {@link Angle} between grid points.
   * @return {@link DoubleJointedArmConfig} for chaining.
   * @throws DoubleJointedArmConfigurationException if the resolution is not above 0.
   */
  public DoubleJointedArmConfig withPathResolution(Angle resolution) {
    Objects.requireNonNull(resolution, "resolution cannot be null");
    if (!(resolution.in(Degrees) > 0)) {
      throw new DoubleJointedArmConfigurationException("Path resolution must be above 0", "Cannot plan around the exclusion zones.",
                                                       "withPathResolution(Angle) above 0");
    }
    this.pathResolution = resolution;
    return this;
  }

  /**
   * Set how close each joint must get to a waypoint before the arm moves on to the next. Paths keep this much following
   * error away from the exclusion zones too, so it must be at least the error the joints' closed loop controllers hold
   * against gravity, or the arm stops on the way. Defaults to half the path resolution.
   *
   * @param tolerance {@link Angle} each joint must be within.
   * @return {@link DoubleJointedArmConfig} for chaining.
   * @throws DoubleJointedArmConfigurationException if the tolerance is not above 0.
   */
  public DoubleJointedArmConfig withPathTolerance(Angle tolerance) {
    Objects.requireNonNull(tolerance, "tolerance cannot be null");
    if (!(tolerance.in(Degrees) > 0)) {
      throw new DoubleJointedArmConfigurationException("Path tolerance must be above 0", "Cannot follow paths around the exclusion zones.",
                                                       "withPathTolerance(Angle) above 0");
    }
    this.pathTolerance = Optional.of(tolerance);
    return this;
  }

  /**
   * Get how close each joint must get to a waypoint before the arm moves on to the next.
   *
   * @return Path tolerance {@link Angle}, half the path resolution unless set.
   */
  public Angle getPathTolerance() {
    return pathTolerance.orElseGet(() -> pathResolution.div(2));
  }

  /**
   * Get the lower (shoulder) {@link ArmConfig}.
   *
   * @return Lower {@link ArmConfig}.
   */
  public ArmConfig getLowerConfig() {
    return lowerConfig;
  }

  /**
   * Get the {@link SmartMotorController} driving the lower joint.
   *
   * @return Lower {@link SmartMotorController}.
   */
  public SmartMotorController getLowerMotorController() {
    return lowerSMC;
  }

  /**
   * Get the upper (elbow) {@link ArmConfig}.
   *
   * @return Upper {@link ArmConfig}.
   */
  public ArmConfig getUpperConfig() {
    return upperConfig;
  }

  /**
   * Get the {@link SmartMotorController} driving the upper joint.
   *
   * @return Upper {@link SmartMotorController}.
   */
  public SmartMotorController getUpperMotorController() {
    return upperSMC;
  }

  /**
   * Get the exclusion zones.
   *
   * @return Unmodifiable list of the {@link ExclusionZone}s.
   */
  public List<ExclusionZone> getExclusionZones() {
    return Collections.unmodifiableList(exclusionZones);
  }

  /**
   * Get the distance the arm keeps from every exclusion zone.
   *
   * @return Clearance {@link Distance}.
   */
  public Distance getExclusionZoneClearance() {
    return clearance;
  }

  /**
   * Get the spacing of the joint space grid paths are planned on.
   *
   * @return Path resolution {@link Angle}.
   */
  public Angle getPathResolution() {
    return pathResolution;
  }

  /**
   * A box the arm must never enter, in the arm's frame. Created from any two opposite corners, which are sorted so
   * {@code min} has the lowest X and Y and {@code max} the highest.
   *
   * @param min Corner with the lowest X and Y, in meters from the shoulder.
   * @param max Corner with the highest X and Y, in meters from the shoulder.
   */
  public record ExclusionZone(Translation2d min, Translation2d max) {
    /** Sort the corners. */
    public ExclusionZone {
      Objects.requireNonNull(min, "min cannot be null");
      Objects.requireNonNull(max, "max cannot be null");
      Translation2d lowest  = new Translation2d(Math.min(min.getX(), max.getX()), Math.min(min.getY(), max.getY()));
      Translation2d highest = new Translation2d(Math.max(min.getX(), max.getX()), Math.max(min.getY(), max.getY()));
      min = lowest;
      max = highest;
    }
  }
}
