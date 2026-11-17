// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;

/** Constants for the Nova arm robot. */
public final class Constants {
  private Constants() {}

  /** The operator's controller. */
  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }

  /** The arm. */
  public static final class ArmConstants {
    private ArmConstants() {}

    /** CAN bus and ID of the Nova. */
    public static final int CAN_BUS = 0;
    public static final int MOTOR_ID = 20;

    /** A 25:1 gearbox, two 5:1 stages, from the NEO to the arm. */
    public static final double FIRST_STAGE_REDUCTION = 5;
    public static final double SECOND_STAGE_REDUCTION = 5;

    /** Simulation only: an 18 in, 2 lb arm. */
    public static final Distance LENGTH = Inches.of(18);
    public static final Mass MASS = Pounds.of(2);

    /** The arm starts level, and can swing from straight down to straight up. */
    public static final Angle START = Degrees.of(0);
    public static final Angle MIN = Degrees.of(-90);
    public static final Angle MAX = Degrees.of(90);

    /** Setpoints. */
    public static final Angle STOW = Degrees.of(-80);
    public static final Angle LEVEL = Degrees.of(0);
    public static final Angle HIGH = Degrees.of(45);
    public static final Angle UP = Degrees.of(80);

    /** Motion profile limits. */
    public static final AngularVelocity MAX_VELOCITY = DegreesPerSecond.of(180);
    public static final AngularAcceleration MAX_ACCELERATION = DegreesPerSecondPerSecond.of(360);

    /** Allowed error to count as at a setpoint. */
    public static final Angle TOLERANCE = Degrees.of(3);
  }
}
