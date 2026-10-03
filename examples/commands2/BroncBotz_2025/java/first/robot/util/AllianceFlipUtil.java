// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.util;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

/** Mirrors blue alliance field positions to the red side when on the red alliance. */
public final class AllianceFlipUtil {
    public static double applyX(double x) {
        return shouldFlip() ? FieldConstants.fieldLength - x : x;
    }

    public static double applyY(double y) {
        return shouldFlip() ? FieldConstants.fieldWidth - y : y;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(applyX(translation.getX()), applyY(translation.getY()));
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return shouldFlip() ? rotation.rotateBy(Rotation2d.PI) : rotation;
    }

    public static Pose2d apply(Pose2d pose) {
        return shouldFlip() ? new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation())) : pose;
    }

    public static boolean shouldFlip() {
        return MatchState.getAlliance().map(alliance -> alliance == Alliance.RED).orElse(false);
    }

    private AllianceFlipUtil() {
    }
}
