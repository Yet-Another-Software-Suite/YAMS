// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Time;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>These are the competition values. The original's {@code DemoMode} outreach values (20 % drive
 * speed, slower shots) were dropped.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
        public static final double kDeadband = 0.05;
    }

    public static class DriveConstants {
        public static final LinearVelocity kMaxSpeed = MetersPerSecond.of(4.5);
        public static final Distance kModuleOffset = Inches.of(13.5);
        // YAGSL's maximum angular velocity: top speed over the distance from center to a module.
        public static final AngularVelocity kMaxAngularSpeed =
            RadiansPerSecond.of(kMaxSpeed.in(MetersPerSecond) / Math.hypot(kModuleOffset.in(Meters), kModuleOffset.in(Meters)));
        public static final double kTranslationScale = 1.0;
        public static final double kRotationScale = 0.8;
        // Driver right bumper, slow mode.
        public static final double kSlowModeTranslationScale = 0.25;
        // Driver A button / autonomous: drive slowly while facing the hub.
        public static final double kAimTranslationScale = 0.3;
    }

    /** Swerve module constants, carried over from the YAGSL JSON configuration. */
    public static class SwerveConstants {
        public static final double kDriveGearRatio = 6.12;
        public static final double kSteerGearRatio = 21.4285714286;
        public static final Distance kWheelDiameter = Inches.of(4);
        public static final double kWheelCircumferenceMeters = Math.PI * kWheelDiameter.in(Meters);

        // YAGSL drive gains are duty cycle per m/s; YAMS's are volts per wheel rotation per second.
        public static final double kDriveKP = 0.0020645 * kWheelCircumferenceMeters * 12;
        // YAGSL's drive feedforward: 12 V at the maximum speed.
        public static final double kDriveKV = 12.0 / (DriveConstants.kMaxSpeed.in(MetersPerSecond) / kWheelCircumferenceMeters);
        // YAGSL steer gains are duty cycle per degree; YAMS's are volts per module rotation.
        public static final double kSteerKP = 0.01 * 360 * 12;

        public static final Current kDriveCurrentLimit = Amps.of(40);
        public static final Current kSteerCurrentLimit = Amps.of(20);
        public static final Time kRampRate = Seconds.of(0.25);

        public static final boolean kDriveInverted = true;
        public static final boolean kSteerInverted = true;

        // Thrifty absolute encoder offsets.
        public static final Angle kFrontLeftEncoderOffset = Degrees.of(225.863);
        public static final Angle kFrontRightEncoderOffset = Degrees.of(216.6);
        public static final Angle kBackLeftEncoderOffset = Degrees.of(134.2676);
        public static final Angle kBackRightEncoderOffset = Degrees.of(57.348);

        // YAGSL heading PID (0.4, 0, 0.01) scales its output by the maximum angular speed.
        public static final double kHeadingKP = 0.4 * DriveConstants.kMaxAngularSpeed.in(RadiansPerSecond);
        public static final double kHeadingKD = 0.01 * DriveConstants.kMaxAngularSpeed.in(RadiansPerSecond);

        // PathPlanner holonomic drive controller gains.
        public static final double kTranslationKP = 5.0;
        public static final double kRotationKP = 5.0;
    }

    public static class ShooterConstants {
        // Operator presets.
        public static final AngularVelocity kCloseShot = RPM.of(1500);
        public static final AngularVelocity kMidShot = RPM.of(2400);
        public static final AngularVelocity kFarShot = RPM.of(3000);
        public static final AngularVelocity kPass = RPM.of(4000);
        // The shooter counts as ready within this much of its target.
        public static final AngularVelocity kReadyTolerance = RPM.of(150);
        // The autonomous shot ends after this long.
        public static final Time kAutoShotTimeout = Seconds.of(6);
    }

    public static class FeedConstants {
        public static final AngularVelocity kKickerSpeed = RPM.of(1792);
        public static final AngularVelocity kIndexerSpeed = RPM.of(1817);
        // Indexer duty cycle while the shooter spins up and while intaking: holds fuel back.
        public static final double kIndexerHold = -0.1;
        public static final double kAgitatorFeed = 0.55;
        public static final double kAgitatorReverse = -0.4;
        // Driver X button.
        public static final double kAgitatorBackOff = -0.2;
        // Operator A button: run the feed path backwards.
        public static final double kUnjamKicker = -0.5;
        public static final double kUnjamIndexer = -0.5;
        public static final double kUnjamAgitator = -0.6;
    }

    public static class IntakeConstants {
        public static final double kRollerIntake = -0.8;
        public static final double kRollerOuttake = 0.8;

        public static final double kArmGearRatio = 9 * 5;
        public static final Distance kArmLength = Inches.of(19.25);
        public static final Mass kArmMass = Pounds.of(11);
        public static final Angle kArmMin = Degrees.of(-3);
        public static final Angle kArmMax = Degrees.of(70);
        // The agitate rock counts as up within this much of kArmUp.
        public static final Angle kAgitateTolerance = Degrees.of(10);

        public static final Angle kArmStart = Degrees.of(67);
        public static final Angle kArmUp = Degrees.of(67);
        public static final Angle kArmIntake = Degrees.of(17);
        public static final Angle kArmDown = Degrees.of(-2);
        // Autonomous wiggle that shakes fuel toward the indexer while shooting.
        public static final Angle kArmWiggleUp = Degrees.of(57);
        public static final Angle kArmWiggleDown = Degrees.of(-5);
        // Operator stick duty cycle scale for manual arm control.
        public static final double kManualArmScale = -0.2;
    }

    /**
     * The hood is driven by a lead screw on a 1:1 NEO, so its "angles" are motor degrees, as in the
     * original.
     */
    public static class HoodConstants {
        public static final Angle kMin = Degrees.of(-1);
        public static final Angle kMax = Degrees.of(21000);
        public static final Angle kDown = Degrees.of(0);
        public static final Angle kShoot = Degrees.of(8965.65);
        public static final Angle kPass = Degrees.of(20108.75);
    }

    private Constants() {
    }
}
