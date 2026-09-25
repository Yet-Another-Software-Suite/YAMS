// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Voltage;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 *
 * <p>
 * Ported from the 2026 Robonauts Everybot. Values are unchanged; they now carry units so YAMS can
 * consume them directly.
 */
public final class Constants {

  public static final class DriveConstants {
    // Motor controller IDs for drivetrain motors
    public static final int LEFT_LEADER_ID = 1;
    public static final int LEFT_FOLLOWER_ID = 3;
    public static final int RIGHT_LEADER_ID = 2;
    public static final int RIGHT_FOLLOWER_ID = 4;

    // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
    // likelihood of tripping breakers or damaging CIM motors
    public static final Current DRIVE_MOTOR_CURRENT_LIMIT = Amps.of(60);

    // Voltage compensation helps the robot perform more similarly on different battery voltages
    public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);

    // Drivetrain gearing and wheel size. The Everybot code never needed these because it drives
    // open loop; YAMS uses them for telemetry and simulation. Update them to match your chassis.
    public static final double DRIVE_GEAR_RATIO = 8.45;
    public static final Distance WHEEL_DIAMETER = Inches.of(6);
  }

  public static final class FuelConstants {
    // Motor controller IDs for Fuel Mechanism motors
    public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
    public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
    public static final int INDEXER_MOTOR_ID = 8;

    // Current limit for fuel mechanism motors.
    public static final Current INDEXER_MOTOR_CURRENT_LIMIT = Amps.of(80);
    public static final Current LAUNCHER_MOTOR_CURRENT_LIMIT = Amps.of(80);

    // All values likely need to be tuned based on your robot. Tune them live with YAMS,
    // then copy the values here.
    public static final double INDEXER_INTAKING_PERCENT = -.8;
    public static final double INDEXER_LAUNCHING_PERCENT = 0.6;
    public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

    public static final double INTAKE_INTAKING_PERCENT = 0.6;
    public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
    public static final double INTAKE_EJECT_PERCENT = -0.8;

    public static final double SPIN_UP_SECONDS = 0.75;

    // Voltage compensation for the launcher rollers
    public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);
  }

  public static final class ClimbConstants {
    // Motor controller IDs for Climb motor
    public static final int CLIMBER_MOTOR_ID = 7;

    // Current limit for climb motor
    public static final Current CLIMBER_MOTOR_CURRENT_LIMIT = Amps.of(40);
    // Percentage to power the motor both up and down
    public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
    public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
  }

  public static final class OperatorConstants {

    // Port constants for driver and operator controllers. These should match the
    // values in the Joystick tab of the Driver Station software
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // This value is multiplied by the joystick value when rotating the robot to
    // help avoid turning too fast and being difficult to control
    public static final double DRIVE_SCALING = 0.7;
    public static final double ROTATION_SCALING = 0.8;
  }
}
