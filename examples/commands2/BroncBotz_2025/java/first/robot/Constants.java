// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Millimeters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>The original kept these in {@code Constants}, {@code Setpoints}, {@code AlignmentConstants} and
 * the YAGSL JSON files; they are merged here.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 4;
        // The Launchpad shows up as three vJoy controllers, created by deploy/launchpad/launch.py.
        public static final int kLaunchpadPort1 = 1;
        public static final int kLaunchpadPort2 = 2;
        public static final int kLaunchpadPort3 = 3;
        public static final double kDeadband = 0.05;
        // Translation scale normally and while the left bumper is held, and the rotation scale.
        public static final double kTranslationScale = 0.8;
        public static final double kSlowTranslationScale = 0.4;
        public static final double kRotationScale = 0.6;
    }

    public static class SwerveConstants {
        // YAGSL was given 7 m/s as the maximum speed, above the 4.9 m/s free speed of a NEO on a 6.12:1
        // drive. Kept so the sticks scale the same.
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(7);
        public static final Distance kModuleOffset = Inches.of(12);
        // YAGSL derives the maximum rotation rate from the maximum speed and the module radius.
        public static final AngularVelocity kMaxAngularSpeed =
            RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) / Math.hypot(kModuleOffset.in(Meters), kModuleOffset.in(Meters)));

        public static final double kDriveGearRatio = 6.12;
        public static final double kAngleGearRatio = 12.8;
        public static final Distance kWheelDiameter = Inches.of(4);
        public static final Current kDriveCurrentLimit = Amps.of(40);
        public static final Current kAngleCurrentLimit = Amps.of(20);
        public static final Time kRampRate = Seconds.of(0.25);

        // NEO free speed, used for the drive feedforward (YAGSL computed the same from its physical
        // properties).
        public static final double kNeoFreeSpeedRps = 5676.0 / 60.0;
        public static final double kWheelFreeSpeedRps = kNeoFreeSpeedRps / kDriveGearRatio;
        // YAGSL's drive kP is per m/s; YAMS closes the loop per wheel rotation per second.
        public static final double kDriveKP = 0.0020645 * Math.PI * kWheelDiameter.in(Meters);
        public static final double kDriveKV = 12.0 / kWheelFreeSpeedRps;
        // YAGSL's angle kP is per degree; YAMS closes the loop per rotation.
        public static final double kAngleKP = 0.01 * 360;

        // Thrifty absolute encoder offsets from the YAGSL module files.
        public static final Angle kFrontLeftEncoderOffset = Degrees.of(84.64);
        public static final Angle kFrontRightEncoderOffset = Degrees.of(284.57);
        public static final Angle kBackLeftEncoderOffset = Degrees.of(7.5);
        public static final Angle kBackRightEncoderOffset = Degrees.of(12.24);

        // YAGSL started the robot here; vision corrects it once tags are seen.
        public static final Pose2d kStartingPose = new Pose2d(Meters.of(10), Meters.of(4), Rotation2d.ZERO);
    }

    /** Drive to pose gains and limits, from the original's {@code AlignmentConstants.DriveToPose}. */
    public static class DriveToPose {
        public static final double kTranslationP = 10;
        public static final double kRotationP = 8;
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(1.3);
        public static final AngularVelocity kMaxAngularSpeed = DegreesPerSecond.of(90);
        public static final Distance kTranslationTolerance = Inches.of(0.3);
        public static final Angle kRotationTolerance = Degrees.of(0.5);
        // Robot relative speed for backing away after scoring.
        public static final LinearVelocity kBackOffSpeed = MetersPerSecond.of(1);
    }

    public static class ElevatorConstants {
        public static final double kGearing = 12.0;
        public static final Distance kChainPitch = Inches.of(0.25);
        public static final int kSprocketTeeth = 22;
        public static final double kP = 33.966;
        public static final double kD = 9.4456;
        public static final double kS = 0.21471; // V
        public static final double kG = 0.23861; // V
        public static final double kV = 10.39; // V/(m/s)
        public static final LinearVelocity kMaxVelocity = MetersPerSecond.of(1);
        public static final LinearAcceleration kMaxAcceleration = MetersPerSecondPerSecond.of(0.5);
        public static final Mass kCarriageMass = Pounds.of(16);
        public static final Distance kMinHeight = Meters.of(0);
        public static final Distance kMaxHeight = Inches.of(30);
        public static final Distance kTolerance = Meters.of(0.07);
        public static final Current kCurrentLimit = Amps.of(40);
        public static final Time kRampRate = Seconds.of(0.1);
        // Duty cycle used to lower onto the bottom stop for L1 and the human player station.
        public static final double kLowerDutyCycle = -0.1;

        public static class Coral {
            public static final Distance L2 = Meters.of(0.014);
            public static final Distance L3 = Meters.of(0.014);
            public static final Distance L4 = Meters.of(0.47);
        }

        public static class Algae {
            public static final Distance L23 = Meters.of(0.039);
            public static final Distance L34 = Meters.of(0.0566);
            public static final Distance NET = Meters.of(0.735);
        }

        // Carriage height used while the arms swing to their stowed angle.
        public static final Distance kAutoClearHeight = Meters.of(0.2);
        public static final Distance kClearHeight = Meters.of(0.3);
    }

    public static class CoralArmConstants {
        public static final double kReduction = 112.0;
        // The original's gains were per motor rotation; YAMS closes the loop per arm rotation.
        public static final double kP = 0.64152 * kReduction;
        public static final double kD = 0.08863 * kReduction;
        public static final double kS = 0.19214; // V
        public static final double kG = 0.023981; // V
        // V per motor rotation per second, converted to V per arm radian per second.
        public static final double kV = 0.11319 * kReduction / (2 * Math.PI);
        // The original's constraints were labeled 10 deg/s and 5 deg/s^2 but were fed RPM into a
        // controller working in rotations per second, so it really ran 600 deg/s and 300 deg/s^2.
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(600);
        public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(300);
        public static final Angle kMinAngle = Degrees.of(-92);
        public static final Angle kMaxAngle = Degrees.of(87);
        public static final Angle kStartingAngle = Degrees.of(-85);
        public static final Angle kTolerance = Degrees.of(3);
        // The through bore encoder reads this when the arm is horizontal.
        public static final Angle kAbsoluteEncoderOffset = Degrees.of(268.6);
        public static final Distance kLength = Inches.of(31);
        public static final Mass kMass = Pounds.of(15);
        public static final Current kCurrentLimit = Amps.of(40);
        public static final Time kRampRate = Seconds.of(0.5);
        public static final boolean kInverted = true;

        public static final Angle HP = Degrees.of(13);
        public static final Angle L1 = Degrees.of(0);
        public static final Angle L2 = Degrees.of(12);
        public static final Angle L3 = Degrees.of(45.14);
        public static final Angle L4 = Degrees.of(67.9);
        public static final Angle kStowed = Degrees.of(-40);
        public static final Angle kAlgaeClearance = Degrees.of(-60);
        public static final Angle kFullyStowed = Degrees.of(-75);
        // How far the arm swings down to pull the coral onto the branch.
        public static final Angle kScoreDrop = Degrees.of(40);

        // LaserCAN readings: coral is loaded between these, and scored once it reads farther than that.
        public static final Distance kLoadedMin = Millimeters.of(45);
        public static final Distance kLoadedMax = Millimeters.of(90);
        public static final Distance kScoredDistance = Millimeters.of(300);
    }

    public static class AlgaeArmConstants {
        public static final double kReduction = 112.0;
        // The original's gains were per motor rotation; YAMS closes the loop per arm rotation.
        public static final double kP = 0.60439 * kReduction;
        public static final double kD = 0.03164 * kReduction;
        public static final double kS = 0.31986; // V
        public static final double kG = 0.16271; // V
        public static final double kV = 0.00091824 * kReduction / (2 * Math.PI);
        // Labeled 20 deg/s and 5 deg/s^2 in the original; see CoralArmConstants.kMaxVelocity.
        public static final AngularVelocity kMaxVelocity = DegreesPerSecond.of(1200);
        public static final AngularAcceleration kMaxAcceleration = DegreesPerSecondPerSecond.of(300);
        public static final Angle kMinAngle = Degrees.of(-78);
        public static final Angle kMaxAngle = Degrees.of(215);
        public static final Angle kStartingAngle = Degrees.of(-60);
        public static final Angle kTolerance = Degrees.of(3);
        public static final Angle kAbsoluteEncoderOffset = Degrees.of(115.2);
        public static final Distance kLength = Inches.of(31);
        public static final Mass kMass = Pounds.of(15);
        public static final Current kCurrentLimit = Amps.of(40);
        public static final Time kRampRate = Seconds.of(0.5);

        public static final Angle L23 = Degrees.of(2.637);
        public static final Angle L34 = Degrees.of(37.2);
        public static final Angle NET = Degrees.of(90);
        public static final Angle PROCESSOR = Degrees.of(-32);
        public static final Angle kStowed = Degrees.of(-40);
        public static final Angle kFullyStowed = Degrees.of(-76);
        // The algae intake only holds the ball while the arm is above this angle.
        public static final Angle kHoldAboveAngle = Degrees.of(-30);
        // How far the arm lifts to pull the algae off the reef.
        public static final Angle kLoadLift = Degrees.of(10);

        public static final Distance kLoadedMin = Millimeters.of(60);
        public static final Distance kLoadedMax = Millimeters.of(140);
        public static final Distance kScoredDistance = Millimeters.of(100);
    }

    public static class CoralIntakeConstants {
        public static final double kWristGearRatio = (30.0 / 54.0) * 28.0;
        public static final double kWristMomentOfInertia = 0.00032; // kg m^2
        public static final double kWristKP = 1;
        public static final Current kWristCurrentLimit = Amps.of(40);
        public static final Time kWristRampRate = Seconds.of(0.25);
        // Wrist positions on the through bore encoder, in rotations.
        public static final Angle kRest = Rotations.of(0.60);
        public static final Angle kActive = Rotations.of(0.35);
        public static final Angle kScoringTolerance = Rotations.of(0.05);

        public static final Current kRollerCurrentLimit = Amps.of(20);
        public static final double kIntake = 0.5;
        public static final double kScore = 0.5;
        public static final double kOuttake = -0.6;
        public static final double kSpit = -0.5;
        public static final double kHold = 0.2;
        public static final double kFull = 1.0;
    }

    public static class AlgaeIntakeConstants {
        public static final Current kCurrentLimit = Amps.of(40);
        public static final double kIntake = 0.8;
        public static final double kOuttake = -0.8;
        public static final double kHold = 0.2;
    }

    /** Where to stand relative to a reef branch, from the original's {@code Setpoints.AutoScoring}. */
    public static class AutoScoring {
        // +X toward the front of the branch pose, +Y to its left.
        public static final Transform2d kCoralOffset =
            new Transform2d(Inches.of(28).in(Meters), Inches.of(7.5).in(Meters), Rotation2d.k180deg);
        public static final Transform2d kAlgaeOffset =
            new Transform2d(Inches.of(24).in(Meters), Inches.of(-14).in(Meters), Rotation2d.k180deg);
    }
}
