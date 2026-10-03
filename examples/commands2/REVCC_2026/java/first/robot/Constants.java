// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Ported from the 2026 REV ION FRC Starter Bot. The REV project kept its SPARK configuration in a
 * separate Configs class; with YAMS that configuration lives in each subsystem's
 * SmartMotorControllerConfig instead, so only IDs, setpoints, and physical dimensions remain here.
 */
public final class Constants {

  public static final class IntakeSubsystemConstants {
    public static final int kIntakeMotorCanId   = 2; // SPARK Flex CAN ID
    public static final int kConveyorMotorCanId = 4; // SPARK Flex CAN ID

    public static final class IntakeSetpoints {
      public static final double kIntake = 0.6;
      public static final double kExtake = -0.6;
    }

    public static final class ConveyorSetpoints {
      public static final double kIntake = 0.7;
      public static final double kExtake = -0.7;
    }
  }

  public static final class ShooterSubsystemConstants {
    public static final int kFeederMotorCanId           = 5; // SPARK Flex CAN ID
    public static final int kFlywheelMotorCanId         = 6; // SPARK Flex CAN ID (Right)
    public static final int kFlywheelFollowerMotorCanId = 7; // SPARK Flex CAN ID (Left)

    public static final class FeederSetpoints {
      public static final double kFeed = 0.95;
    }

    public static final class FlywheelSetpoints {
      public static final AngularVelocity kShootRpm          = RPM.of(5000);
      public static final AngularVelocity kVelocityTolerance = RPM.of(100);
    }
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final LinearVelocity  kMaxSpeed        = MetersPerSecond.of(4.8);
    public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(360);

    // Chassis configuration
    // Distance between centers of right and left wheels on robot
    public static final Distance kTrackWidth = Inches.of(22.5);
    // Distance between front and back wheels on robot
    public static final Distance kWheelBase  = Inches.of(22.5);

    // Angular offsets of the modules relative to the chassis. REV subtracts these from the Through
    // Bore reading; YAMS applies them as the SPARK absolute encoder zero offset, which must be in
    // [0, 360) degrees, so the REV values (-pi/4, pi/4, 5pi/4, 3pi/4) are wrapped into that range.
    public static final Angle kFrontLeftChassisAngularOffset  = Degrees.of(315);
    public static final Angle kFrontRightChassisAngularOffset = Degrees.of(45);
    public static final Angle kBackLeftChassisAngularOffset   = Degrees.of(225);
    public static final Angle kBackRightChassisAngularOffset  = Degrees.of(135);

    // The EasySwerve module allows installation of the motors either on top or bottom of the module.
    // These constants configure the location of the motors. The default configuration is with both
    // motors on the bottom of the module.
    public static final boolean kFrontLeftDrivingMotorOnBottom  = true;
    public static final boolean kRearLeftDrivingMotorOnBottom   = true;
    public static final boolean kFrontRightDrivingMotorOnBottom = true;
    public static final boolean kRearRightDrivingMotorOnBottom  = true;

    public static final boolean kFrontLeftTurningMotorOnBottom  = true;
    public static final boolean kRearLeftTurningMotorOnBottom   = true;
    public static final boolean kFrontRightTurningMotorOnBottom = true;
    public static final boolean kRearRightTurningMotorOnBottom  = true;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId  = 15;
    public static final int kRearLeftDrivingCanId   = 13;
    public static final int kFrontRightDrivingCanId = 11;
    public static final int kRearRightDrivingCanId  = 9;

    public static final int kFrontLeftTurningCanId  = 14;
    public static final int kRearLeftTurningCanId   = 12;
    public static final int kFrontRightTurningCanId = 10;
    public static final int kRearRightTurningCanId  = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class NeoMotorConstants {
    public static final AngularVelocity kFreeSpeed       = RPM.of(5676);
    public static final AngularVelocity kVortexFreeSpeed = RPM.of(6784);
  }

  public static final class ModuleConstants {
    // The EasySwerve module can only be configured with one pinion gears: 12T.
    public static final int kDrivingMotorPinionTeeth = 12;

    public static final Distance kWheelDiameter = Inches.of(4);
    // 45 teeth on the wheel's bevel gear, 30 teeth on the first-stage spur gear,
    // 15 teeth on the bevel pinion
    public static final double kDrivingWheelBevelGearTeeth          = 45.0;
    public static final double kDrivingWheelFirstStageSpurGearTeeth = 30.0;
    public static final double kDrivingMotorBevelPinionTeeth        = 15.0;
    public static final double kDrivingMotorReduction               =
        (kDrivingWheelBevelGearTeeth * kDrivingWheelFirstStageSpurGearTeeth)
        / (kDrivingMotorPinionTeeth * kDrivingMotorBevelPinionTeeth);

    // Steering motor to module output reduction. The REV code never needed this because it closes
    // the steering loop directly on the Through Bore Encoder; YAMS uses it for the relative encoder
    // and the simulator.
    public static final double kTurningMotorReduction = 20.0;
  }

  public static final class OIConstants {
    public static final int    kDriverControllerPort   = 0;
    public static final double kDriveDeadband          = 0.1;
    public static final double kTriggerButtonThreshold = 0.2;
  }
}
