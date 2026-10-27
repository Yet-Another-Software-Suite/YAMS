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
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Double jointed arm configuration: the lower (shoulder) and upper (elbow) arms, and the exclusion zones the arm must
 * never enter while it travels.
 *
 * <p>The lower arm's angle is measured from horizontal. The upper arm's angle is the upper motor controller's, which is
 * measured from horizontal by default, as on an arm whose elbow is driven from the base, e.g. by a chain through the
 * shoulder, or from the lower arm with {@link #withUpperAngleReference(UpperAngleReference)}, as on an arm with its elbow
 * motor on the lower arm. The arm's frame has its origin at the shoulder, with X away from the shoulder and Y up, the
 * same as {@link DoubleJointedArm#getPosition()} and
 * {@link DoubleJointedArm#getAnglesForPosition(Translation2d, boolean)}.
 *
 * <p>Gravity on each joint depends on where both arms are, which an {@link org.wpilib.math.controller.ArmFeedforward}
 * can't know, so {@link DoubleJointedArm} changes each joint's kG as the arm moves. Tune each kG with the arm held
 * straight out horizontally; see {@link #withKgUpdateThreshold(double)}.
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
  /** What the upper arm's angle is measured from. */
  private UpperAngleReference        upperAngleReference = UpperAngleReference.HORIZONTAL;
  /** Fraction kG must change by before it is sent to a motor controller again. */
  private double                     kGUpdateThreshold   = 0.05;
  /** Telemetry name. */
  private Optional<String>           telemetryName       = Optional.empty();
  /** Telemetry verbosity. */
  private Optional<TelemetryVerbosity> telemetryVerbosity = Optional.empty();

  /** What the upper arm's angle, as its motor controller reports it, is measured from. */
  public enum UpperAngleReference {
    /**
     * From horizontal, so it doesn't change when only the lower arm moves: an elbow driven from the base, e.g. by a
     * chain through the shoulder. The elbow motor's torque then acts between the base and the upper arm.
     */
    HORIZONTAL,
    /**
     * From the lower arm, so it is 0 with the arm straight: an elbow motor on the lower arm. The elbow motor's torque
     * then acts between the lower and upper arms.
     */
    LOWER_ARM
  }

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
   * Set what the upper arm's angle, as its motor controller reports it, is measured from. Defaults to
   * {@link UpperAngleReference#HORIZONTAL}.
   *
   * @param reference {@link UpperAngleReference} of the upper arm's angle.
   * @return {@link DoubleJointedArmConfig} for chaining.
   */
  public DoubleJointedArmConfig withUpperAngleReference(UpperAngleReference reference) {
    this.upperAngleReference = Objects.requireNonNull(reference, "reference cannot be null");
    return this;
  }

  /**
   * Set how much each joint's kG must change, as a fraction, before it is sent to its motor controller again. Each send
   * reconfigures the motor controller, so small changes are left until they add up. Defaults to 0.05.
   *
   * <p>Each joint's kG is scaled from the one its {@link org.wpilib.math.controller.ArmFeedforward} was configured or
   * last live tuned with, by how much gravity on the joint has changed from when the arm is held straight out
   * horizontally, as a uniform rod model of each arm, from its motor controller's moment of inertia, works it out:
   *
   * <ul>
   *   <li>With the upper arm's angle from {@link UpperAngleReference#HORIZONTAL horizontal}, the upper arm's own weight
   *   is carried from the base, so gravity on each joint only depends on its own angle, as an ArmFeedforward assumes,
   *   and kG never changes.</li>
   *   <li>With it from {@link UpperAngleReference#LOWER_ARM the lower arm}, the shoulder also carries the upper arm's
   *   weight wherever it points, and the elbow's gravity depends on the angle of the upper arm from horizontal, not
   *   from the lower arm.</li>
   * </ul>
   *
   * <p>Near a joint pointing straight up or down its ArmFeedforward gives almost nothing whatever kG is, so kG is left
   * as it was there.
   *
   * @param fraction Change in kG, as a fraction of it, before it is sent again.
   * @return {@link DoubleJointedArmConfig} for chaining.
   * @throws DoubleJointedArmConfigurationException if the fraction is negative.
   */
  public DoubleJointedArmConfig withKgUpdateThreshold(double fraction) {
    if (!(fraction >= 0)) {
      throw new DoubleJointedArmConfigurationException("kG update threshold is negative", "Cannot compensate kG for the arm's position.",
                                                       "withKgUpdateThreshold(double) with 0 or more");
    }
    this.kGUpdateThreshold = fraction;
    return this;
  }

  /**
   * Publish the arm's own telemetry: where the end of the arm and the elbow are, the path it is following, each joint's
   * kG, and at {@link TelemetryVerbosity#HIGH} a live tunable path tolerance.
   *
   * @param name      Telemetry name, which also names the arm.
   * @param verbosity {@link TelemetryVerbosity} to publish at.
   * @return {@link DoubleJointedArmConfig} for chaining.
   */
  public DoubleJointedArmConfig withTelemetry(String name, TelemetryVerbosity verbosity) {
    this.telemetryName = Optional.of(Objects.requireNonNull(name, "name cannot be null"));
    this.telemetryVerbosity = Optional.of(Objects.requireNonNull(verbosity, "verbosity cannot be null"));
    return this;
  }

  /**
   * Get what the upper arm's angle is measured from.
   *
   * @return {@link UpperAngleReference}.
   */
  public UpperAngleReference getUpperAngleReference() {
    return upperAngleReference;
  }

  /**
   * Get how much each joint's kG must change, as a fraction, before it is sent to its motor controller again.
   *
   * @return kG update threshold.
   */
  public double getKgUpdateThreshold() {
    return kGUpdateThreshold;
  }

  /**
   * Get the telemetry name.
   *
   * @return Telemetry name, or empty without the arm's own telemetry.
   */
  public Optional<String> getTelemetryName() {
    return telemetryName;
  }

  /**
   * Get the telemetry verbosity.
   *
   * @return {@link TelemetryVerbosity}, or empty without the arm's own telemetry.
   */
  public Optional<TelemetryVerbosity> getTelemetryVerbosity() {
    return telemetryVerbosity;
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
