// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Rotations;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class DriveConstants {
    public static final int kLeftFrontId = 4;
    public static final int kLeftRearId = 3;
    public static final int kRightFrontId = 2;
    public static final int kRightRearId = 1;

    // The original drives open loop and never needed these; YAMS uses them for telemetry and
    // simulation. Update them to match your chassis.
    public static final double kGearRatio = 8.45;
    public static final Distance kWheelDiameter = Inches.of(6);
  }

  public static class ArmConstants {
    public static final int kLeaderId = 6;
    public static final int kFollowerId = 7;
    public static final int kEncoderChannel = 0;

    // Define Arm positions, in rotations of the through bore encoder
    public static final Angle positionIntakeCoral      = Rotations.of(0.422);
    public static final Angle positionClimbEnd         = Rotations.of(0.368);
    public static final Angle positionIntakeAlgae      = Rotations.of(0.348);
    public static final Angle positionRemoveAlgaeLow   = Rotations.of(0.3083);
    public static final Angle positionClimbStart       = Rotations.of(0.233);
    public static final Angle positionRemoveAlgaeHigh  = Rotations.of(0.1);

    // Define Arm position limits
    public static final Angle armFrontLimit            = Rotations.of(0.422);
    public static final Angle armRearLimit             = Rotations.of(0.05);

    // Arm PID gains from the original are duty cycle per rotation; YAMS outputs volts, so they are
    // scaled by 12 V here.
    public static final double armkP                   = 17.5 * 12.0;
    public static final double armkI                   = 0.0;
    public static final double armkD                   = 0.8 * 12.0;

    // Motor to arm reduction. The arm closes the loop on the motor encoder after it is seeded from the
    // through bore, so this must match your arm. 100 is a placeholder.
    public static final double kGearRatio = 100.0;
  }

  public static class IntakeConstants {
    public static final int kLeaderId = 8;
    public static final int kFollowerId = 9;
  }

  public static class ClimberConstants {
    public static final int kMotorId = 10;
  }
}
