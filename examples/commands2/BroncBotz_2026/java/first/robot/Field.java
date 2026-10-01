// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/**
 * The few field positions the robot uses, blue alliance origin, and the alliance flip. Replaces the
 * original's {@code FieldConstants} and {@code AllianceFlipUtil}, which loaded the whole 2026
 * AprilTag layout to read these values.
 *
 * <p>The 2026 field is rotationally symmetric, so the red alliance's positions are the blue ones
 * rotated 180 degrees about the field center, as PathPlanner flipped them.
 */
public final class Field {
    // Welded 2026 field, from the official AprilTag layout.
    public static final double kLengthMeters = 16.541;
    public static final double kWidthMeters = 8.069;

    /** Blue hub center. */
    public static final Translation2d kBlueHub = new Translation2d(Inches.of(182.11), Inches.of(158.84));

    /**
     * Blue pose with the robot's back against the near hub face (AprilTag 26), half a 32 in robot
     * length out from it. Driver D-pad up resets odometry here.
     */
    public static final Pose2d kBlueRobotPoseAtHub = new Pose2d(4.0218614 - Inches.of(16).in(Meters), 4.0346376, Rotation2d.ZERO);

    private Field() {
    }

    /** Whether the robot is on the red alliance. An unknown alliance counts as blue. */
    public static boolean isRed() {
        return MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
    }

    /** Rotate a blue position 180 degrees about the field center. */
    public static Translation2d flip(Translation2d translation) {
        return new Translation2d(kLengthMeters - translation.getX(), kWidthMeters - translation.getY());
    }

    /** Rotate a blue pose 180 degrees about the field center. */
    public static Pose2d flip(Pose2d pose) {
        return new Pose2d(flip(pose.getTranslation()), pose.getRotation().rotateBy(Rotation2d.k180deg));
    }

    /** Mirror a pose across the field's long centerline, like a mirrored PathPlanner auto. */
    public static Pose2d mirror(Pose2d pose) {
        return new Pose2d(pose.getX(), kWidthMeters - pose.getY(), pose.getRotation().unaryMinus());
    }

    /** A blue pose, flipped when on the red alliance. */
    public static Pose2d forAlliance(Pose2d bluePose) {
        return isRed() ? flip(bluePose) : bluePose;
    }

    /** Center of this alliance's hub. */
    public static Translation2d hub() {
        return isRed() ? flip(kBlueHub) : kBlueHub;
    }

    /** Distance from a robot pose to this alliance's hub, in meters. */
    public static double distanceToHub(Pose2d robotPose) {
        return robotPose.getTranslation().getDistance(hub());
    }
}
