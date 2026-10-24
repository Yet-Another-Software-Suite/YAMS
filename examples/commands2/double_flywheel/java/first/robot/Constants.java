// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

/** Constants for the double flywheel robot. */
public final class Constants {
  private Constants() {}

  /** The double flywheel. */
  public static final class DoubleFlyWheelConstants {
    private DoubleFlyWheelConstants() {}

    public static final int LOWER_MOTOR_ID = 10;
    public static final int UPPER_MOTOR_ID = 11;
    public static final Distance WHEEL_DIAMETER = Inches.of(4);

    /** A shot with backspin: the lower wheel runs faster than the upper one. */
    public static final AngularVelocity SHOT_LOWER_SPEED = RPM.of(3000);
    public static final AngularVelocity SHOT_UPPER_SPEED = RPM.of(2000);
    /** Both wheels' allowed error before the shot is ready. */
    public static final AngularVelocity READY_TOLERANCE = RPM.of(100);
  }

  /** The operator's controller. */
  public static final class OperatorConstants {
    private OperatorConstants() {}

    public static final int OPERATOR_CONTROLLER_PORT = 0;
  }
}
