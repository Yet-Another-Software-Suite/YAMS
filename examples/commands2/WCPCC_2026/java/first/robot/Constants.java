// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class Driving {
        public static final LinearVelocity kMaxSpeed = SwerveConstants.kSpeedAt12Volts;
        public static final AngularVelocity kMaxRotationalRate = RotationsPerSecond.of(1);
        // Joystick deadband; SwerveInputStream also cubes the translation and rotation axes.
        public static final double kJoystickDeadband = 0.15;
        // Rotation stick must be idle this long before the current heading is held.
        public static final double kHeadingLockDelaySeconds = 0.25;
    }

    public static class KrakenX60 {
        public static final AngularVelocity kFreeSpeed = RPM.of(6000);
    }

    /**
     * Swerve module constants. These replace the Phoenix Tuner X generated TunerConstants, which only
     * applied to CTRE's own swerve API; YAMS builds the drivetrain from the same values.
     */
    public static class SwerveConstants {
        public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(5.42);

        public static final double kDriveGearRatio = 5.8909090909090915;
        public static final double kSteerGearRatio = 12.1;
        // Steer to drive coupling. YAMS does not compensate for coupling yet, so this is unused.
        public static final double kCoupleRatio = 4.909090909090909;
        public static final Distance kWheelRadius = Inches.of(2);

        // Drive gains from Tuner X are per motor rotation; YAMS closes the loop per wheel rotation, so
        // they are scaled by the drive gear ratio where they are used.
        public static final double kDriveKP = 0.1;
        public static final double kDriveKV = 0.124;

        // Steer gains are already per module rotation (fused CANcoder feedback).
        public static final double kSteerKP = 100;
        public static final double kSteerKD = 0.5;
        public static final double kSteerKS = 0.1;
        public static final double kSteerKV = 1.16;

        public static final Current kSlipCurrent = Amps.of(120.0);
        public static final Current kSteerStatorCurrentLimit = Amps.of(60);

        public static final boolean kInvertLeftSide = false;
        public static final boolean kInvertRightSide = true;
        public static final boolean kSteerMotorInverted = true;

        public static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.172607421875);
        public static final Angle kFrontRightEncoderOffset = Rotations.of(0.337890625);
        public static final Angle kBackLeftEncoderOffset = Rotations.of(-0.01171875);
        public static final Angle kBackRightEncoderOffset = Rotations.of(0.326171875);

        // Module locations: +X toward the front of the robot, +Y toward the left.
        public static final Distance kModuleOffset = Inches.of(10);
    }
}
