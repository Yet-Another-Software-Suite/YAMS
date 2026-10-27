// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Radians;

import java.util.EnumMap;
import java.util.Map;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.util.struct.Struct;
import yams.core.math.DoubleJointedArmPath;
import yams.core.mechanisms.positional.DoubleJointedArm;
import yams.core.mechanisms.positional.DoubleJointedArmPathFollower;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Double jointed arm telemetry, under the arm's {@code Mechanisms/<name>} and {@code Tuning/<name>} tables.
 *
 * <ul>
 *   <li>{@link TelemetryVerbosity#LOW}: where the end of the arm is, and whether the arm is in an exclusion zone.</li>
 *   <li>{@link TelemetryVerbosity#MID}: also where the elbow is, the upper arm's angle from horizontal, the goal and
 *   waypoint the arm is moving to and how many waypoints its path has, each joint's kG as compensated for where the arm
 *   is, and the path tolerance.</li>
 *   <li>{@link TelemetryVerbosity#HIGH}: also the path tolerance in the Tuning table, which the arm's
 *   {@code Live Tuning} command applies while it runs.</li>
 * </ul>
 *
 * <p>Angles are in degrees, and positions in meters from the shoulder.
 *
 * <p>Managed by {@link DoubleJointedArm}; enable it with
 * {@link yams.core.mechanisms.config.DoubleJointedArmConfig#withTelemetry(String, TelemetryVerbosity)}.
 */
public class DoubleJointedArmTelemetry {
  /** Double telemetry fields. */
  private final Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>>       doubleFields = new EnumMap<>(DoubleTelemetryField.class);
  /** Boolean telemetry fields. */
  private final Map<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>>    boolFields   = new EnumMap<>(BooleanTelemetryField.class);
  /** Struct telemetry fields. */
  private final Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>>     structFields = new EnumMap<>(StructTelemetryField.class);

  /**
   * Create and publish double jointed arm telemetry.
   *
   * @param verbosity   {@link TelemetryVerbosity} to publish at.
   * @param dataTable   Table to publish to.
   * @param tuningTable Table to publish the tunable fields to.
   * @param arm         {@link DoubleJointedArm} to publish, for the tunable fields' starting values.
   */
  public DoubleJointedArmTelemetry(TelemetryVerbosity verbosity, NetworkTable dataTable, NetworkTable tuningTable, DoubleJointedArm arm) {
    for (DoubleTelemetryField field : DoubleTelemetryField.values()) {
      doubleFields.put(field, field.create());
    }
    for (BooleanTelemetryField field : BooleanTelemetryField.values()) {
      boolFields.put(field, field.create());
    }
    for (StructTelemetryField field : StructTelemetryField.values()) {
      structFields.put(field, field.create());
    }
    switch (verbosity) {
      case HIGH:
        doubleFields.get(DoubleTelemetryField.TunablePathTolerance).enable();
      case MID:
        structFields.get(StructTelemetryField.Elbow).enable();
        doubleFields.get(DoubleTelemetryField.UpperAngleFromHorizontal).enable();
        doubleFields.get(DoubleTelemetryField.GoalLower).enable();
        doubleFields.get(DoubleTelemetryField.GoalUpper).enable();
        doubleFields.get(DoubleTelemetryField.WaypointLower).enable();
        doubleFields.get(DoubleTelemetryField.WaypointUpper).enable();
        doubleFields.get(DoubleTelemetryField.WaypointIndex).enable();
        doubleFields.get(DoubleTelemetryField.Waypoints).enable();
        doubleFields.get(DoubleTelemetryField.LowerKg).enable();
        doubleFields.get(DoubleTelemetryField.UpperKg).enable();
        doubleFields.get(DoubleTelemetryField.PathTolerance).enable();
        boolFields.get(BooleanTelemetryField.HasPath).enable();
      case LOW:
        structFields.get(StructTelemetryField.Position).enable();
        boolFields.get(BooleanTelemetryField.InExclusionZone).enable();
    }
    doubleFields.get(DoubleTelemetryField.TunablePathTolerance).setDefaultValue(arm.getPathTolerance().in(Degrees));
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (dt.enabled) {
        dt.setupNetworkTables(dataTable, tuningTable);
      }
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      if (bt.enabled) {
        bt.setupNetworkTables(dataTable, tuningTable);
      }
    }
    for (StructTelemetry<?, StructTelemetryField> st : structFields.values()) {
      if (st.enabled) {
        st.setupNetworkTables(dataTable, tuningTable);
      }
    }
    // Replace a tolerance an earlier telemetry left on the dashboard with the arm's, rather than apply it.
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      dt.tunable();
    }
    applyTuningValues(arm);
  }

  /**
   * Publish the arm's telemetry.
   *
   * @param arm {@link DoubleJointedArm} to publish.
   */
  @SuppressWarnings("unchecked")
  public void publish(DoubleJointedArm arm) {
    final DoubleJointedArmPathFollower follower = arm.getPathFollower();
    final var                          path     = follower.getPath();
    final int                          index    = follower.getWaypointIndex();
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (!dt.enabled) {
        continue;
      }
      switch (dt.getField()) {
        case UpperAngleFromHorizontal -> dt.set(arm.getUpperAngleFromHorizontal().in(Degrees));
        case GoalLower -> dt.set(Math.toDegrees(follower.getGoalLowerRadians()));
        case GoalUpper -> dt.set(Math.toDegrees(follower.getGoalUpperRadians()));
        case WaypointLower -> dt.set(path.map(waypoints -> waypoints.getLowerAngle(index).in(Degrees)).orElse(Double.NaN));
        case WaypointUpper -> dt.set(path.map(waypoints -> waypoints.getUpperAngle(index).in(Degrees)).orElse(Double.NaN));
        case WaypointIndex -> dt.set(path.isPresent() ? index : Double.NaN);
        case Waypoints -> dt.set(path.map(DoubleJointedArmPath::size).orElse(0));
        case LowerKg -> dt.set(arm.getKg(0));
        case UpperKg -> dt.set(arm.getKg(1));
        case PathTolerance -> dt.set(arm.getPathTolerance().in(Degrees));
        default -> {
          // Tunable fields are applied by applyTuningValues.
        }
      }
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      if (!bt.enabled) {
        continue;
      }
      switch (bt.getField()) {
        case InExclusionZone -> bt.set(arm.isInExclusionZone());
        case HasPath -> bt.set(path.isPresent());
      }
    }
    for (StructTelemetry<?, StructTelemetryField> st : structFields.values()) {
      if (!st.enabled) {
        continue;
      }
      switch (st.getField()) {
        case Position -> ((StructTelemetry<Translation2d, StructTelemetryField>) st).set(arm.getPosition());
        case Elbow -> ((StructTelemetry<Translation2d, StructTelemetryField>) st).set(arm.getElbowPosition());
      }
    }
  }

  /**
   * Apply the path tolerance edited on the dashboard to the arm, and publish the arm's tolerance when it was changed in
   * code or the edit was invalid. The arm's {@code Live Tuning} command calls this every loop.
   *
   * @param arm {@link DoubleJointedArm} to tune.
   */
  public void applyTuningValues(DoubleJointedArm arm) {
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      if (!dt.enabled) {
        continue;
      }
      switch (dt.getField()) {
        case TunablePathTolerance -> {
          if (dt.tunable() && dt.get() > 0 && Double.isFinite(dt.get())) {
            arm.setPathTolerance(Degrees.of(dt.get()));
          }
          final double tolerance = arm.getPathTolerance().in(Degrees);
          if (dt.get() != tolerance) {
            dt.forceSet(tolerance);
            // Seen, so it isn't applied back to the arm as an edit.
            dt.tunable();
          }
        }
        default -> {
          // Read-only fields are published by publish.
        }
      }
    }
  }

  /** Close and unpublish the telemetry. */
  public void close() {
    for (DoubleTelemetry<DoubleTelemetryField> dt : doubleFields.values()) {
      dt.close();
    }
    for (BooleanTelemetry<BooleanTelemetryField> bt : boolFields.values()) {
      bt.close();
    }
    for (StructTelemetry<?, StructTelemetryField> st : structFields.values()) {
      st.close();
    }
  }

  /** Double telemetry field for {@link DoubleJointedArm}s. */
  public enum DoubleTelemetryField {
    /** Upper arm's angle from horizontal, whatever its joint angle is measured from. */
    UpperAngleFromHorizontal("upperAngleFromHorizontal", 0, false, "degrees"),
    /** Goal lower joint angle, NaN before a goal is set. */
    GoalLower("path/goal/lower", Double.NaN, false, "degrees"),
    /** Goal upper joint angle, NaN before a goal is set. */
    GoalUpper("path/goal/upper", Double.NaN, false, "degrees"),
    /** Lower joint angle of the waypoint being moved to, NaN without a path. */
    WaypointLower("path/waypoint/lower", Double.NaN, false, "degrees"),
    /** Upper joint angle of the waypoint being moved to, NaN without a path. */
    WaypointUpper("path/waypoint/upper", Double.NaN, false, "degrees"),
    /** Index of the waypoint being moved to, NaN without a path. */
    WaypointIndex("path/waypoint/index", Double.NaN, false, "none"),
    /** Waypoints in the path, 0 without one. */
    Waypoints("path/waypoints", 0, false, "none"),
    /** Lower joint's kG, compensated for where the arm is, NaN without an ArmFeedforward. */
    LowerKg("lower/kG", Double.NaN, false, "volts"),
    /** Upper joint's kG, compensated for where the arm is, NaN without an ArmFeedforward. */
    UpperKg("upper/kG", Double.NaN, false, "volts"),
    /** How close each joint must get to a waypoint before the arm moves on. */
    PathTolerance("path/tolerance", 0, false, "degrees"),
    /** Tunable path tolerance. */
    TunablePathTolerance("path/tolerance", 0, true, "degrees");

    /** Default value of the double telemetry field. */
    private final double  defaultVal;
    /** Key that the telemetry is stored at. */
    private final String  key;
    /** Tunable field? */
    private final boolean tunable;
    /** Unit of the telemetry field. */
    private final String  unit;

    /**
     * Create a double telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     * @param unit         Unit of the telemetry field.
     */
    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable, String unit) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
      this.unit = unit;
    }

    /**
     * Create a {@link DoubleTelemetry} object for non-static usage.
     *
     * @return {@link DoubleTelemetry}
     */
    public DoubleTelemetry<DoubleTelemetryField> create() {
      return new DoubleTelemetry<>(key, defaultVal, this, tunable, unit);
    }
  }

  /** Boolean telemetry field for {@link DoubleJointedArm}s. */
  public enum BooleanTelemetryField {
    /** The arm is in an exclusion zone, or closer to one than the clearance. */
    InExclusionZone("inExclusionZone", false, false),
    /** The arm has a path to its goal, rather than holding where it is. */
    HasPath("path/hasPath", false, false);

    /** Key that the telemetry is stored at. */
    private final String  key;
    /** Default value of the boolean telemetry field. */
    private final boolean defaultVal;
    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a boolean telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
     *
     * @return {@link BooleanTelemetry}
     */
    public BooleanTelemetry<BooleanTelemetryField> create() {
      return new BooleanTelemetry<>(key, defaultVal, this, tunable);
    }
  }

  /** Struct telemetry field for {@link DoubleJointedArm}s. */
  public enum StructTelemetryField {
    /** Where the end of the arm is, from the shoulder. */
    Position("position", Translation2d.struct, Translation2d.ZERO, false),
    /** Where the elbow is, from the shoulder. */
    Elbow("elbow", Translation2d.struct, Translation2d.ZERO, false);

    /** Key that the telemetry is stored at. */
    private final String    key;
    /** {@link Struct} serializer for the field's value type. */
    private final Struct<?> struct;
    /** Default value of the struct telemetry field. */
    private final Object    defaultValue;
    /** Tunable field? */
    private final boolean   tunable;

    /**
     * Create a struct telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param struct       {@link Struct} serializer for the field's value type {@link T}.
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     * @param <T>          Type of the field's value.
     */
    <T> StructTelemetryField(String fieldName, Struct<T> struct, T defaultValue, boolean tunable) {
      key = fieldName;
      this.struct = struct;
      this.defaultValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StructTelemetry} object for non-static usage.
     *
     * @param <T> Type of the field's value, must match the type this field was declared with.
     * @return {@link StructTelemetry}
     */
    @SuppressWarnings("unchecked")
    public <T> StructTelemetry<T, StructTelemetryField> create() {
      return new StructTelemetry<>(key, (T) defaultValue, this, (Struct<T>) struct, tunable);
    }
  }
}
