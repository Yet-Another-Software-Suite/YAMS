// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.swerve;

import static org.wpilib.units.Units.MetersPerSecond;

import java.util.Objects;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;

/**
 * Telemetry and live tuning support for {@link SwerveInputStream}.
 */
public class SwerveInputStreamTelemetry {
  private final SwerveInputStream stream;
  private final NetworkTable table;

  public SwerveInputStreamTelemetry(SwerveInputStream stream, String name) {
    this.stream = Objects.requireNonNull(stream, "stream cannot be null");
    this.table = NetworkTableInstance.getDefault().getTable("SwerveInputStream").getSubTable(name);
  }

  public void update() {
    ChassisVelocities speeds = stream.get();
    table.getStringTopic("mode").getEntry("UNKNOWN").set(stream.getCurrentModeName());
    table.getDoubleTopic("vx").getEntry(0.0).set(speeds.vx);
    table.getDoubleTopic("vy").getEntry(0.0).set(speeds.vy);
    table.getDoubleTopic("omega").getEntry(0.0).set(speeds.omega);

    updateDeadband();
    updateTranslationScale();
    updateRotationScale();
    updateMaxLinearVelocity();
    updateMaxAngularVelocity();
    updateTranslationCube();
    updateRotationCube();
    updateAllianceRelative();
    updateRobotRelative();
  }

  private void updateDeadband() {
    var entry = table.getDoubleTopic("deadband").getEntry(stream.getAxisDeadband());
    entry.set(stream.getAxisDeadband());
    double tuned = entry.get();
    if (tuned != stream.getAxisDeadband()) {
      stream.setAxisDeadband(tuned);
    }
  }

  private void updateTranslationScale() {
    var entry = table.getDoubleTopic("translationScale").getEntry(stream.getTranslationAxisScale());
    entry.set(stream.getTranslationAxisScale());
    double tuned = entry.get();
    if (tuned != stream.getTranslationAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      stream.setTranslationAxisScale(tuned);
    }
  }

  private void updateRotationScale() {
    var entry = table.getDoubleTopic("rotationScale").getEntry(stream.getOmegaAxisScale());
    entry.set(stream.getOmegaAxisScale());
    double tuned = entry.get();
    if (tuned != stream.getOmegaAxisScale() && tuned > 0.0 && tuned <= 1.0) {
      stream.setOmegaAxisScale(tuned);
    }
  }

  private void updateMaxLinearVelocity() {
    var entry = table.getDoubleTopic("maxLinearVelocity").getEntry(stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
    entry.set(stream.getMaximumChassisLinearVelocity().in(MetersPerSecond));
    double tuned = entry.get();
    if (tuned != stream.getMaximumChassisLinearVelocity().in(MetersPerSecond) && tuned > 0.0) {
      stream.setMaximumChassisLinearVelocity(MetersPerSecond.of(tuned));
    }
  }

  private void updateMaxAngularVelocity() {
    var entry = table.getDoubleTopic("maxAngularVelocity").getEntry(stream.getMaximumChassisAngularVelocity().in(org.wpilib.units.Units.RadiansPerSecond));
    entry.set(stream.getMaximumChassisAngularVelocity().in(org.wpilib.units.Units.RadiansPerSecond));
    double tuned = entry.get();
    if (tuned != stream.getMaximumChassisAngularVelocity().in(org.wpilib.units.Units.RadiansPerSecond) && tuned > 0.0) {
      stream.setMaximumChassisAngularVelocity(org.wpilib.units.Units.RadiansPerSecond.of(tuned));
    }
  }

  private void updateTranslationCube() {
    var entry = table.getBooleanTopic("translationCube").getEntry(stream.isTranslationCubeEnabled());
    entry.set(stream.isTranslationCubeEnabled());
    boolean tuned = entry.get();
    if (tuned != stream.isTranslationCubeEnabled()) {
      stream.setTranslationCubeEnabled(tuned);
    }
  }

  private void updateRotationCube() {
    var entry = table.getBooleanTopic("rotationCube").getEntry(stream.isOmegaCubeEnabled());
    entry.set(stream.isOmegaCubeEnabled());
    boolean tuned = entry.get();
    if (tuned != stream.isOmegaCubeEnabled()) {
      stream.setOmegaCubeEnabled(tuned);
    }
  }

  private void updateAllianceRelative() {
    var entry = table.getBooleanTopic("allianceRelative").getEntry(stream.isAllianceRelativeEnabled());
    entry.set(stream.isAllianceRelativeEnabled());
    boolean tuned = entry.get();
    if (tuned != stream.isAllianceRelativeEnabled()) {
      stream.setAllianceRelativeEnabled(tuned);
    }
  }

  private void updateRobotRelative() {
    var entry = table.getBooleanTopic("robotRelative").getEntry(stream.isRobotRelativeEnabled());
    entry.set(stream.isRobotRelativeEnabled());
    boolean tuned = entry.get();
    if (tuned != stream.isRobotRelativeEnabled()) {
      stream.setRobotRelativeEnabled(tuned);
    }
  }
}
