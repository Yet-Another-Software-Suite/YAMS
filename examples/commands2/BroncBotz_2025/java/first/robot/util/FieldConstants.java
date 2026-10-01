// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.
//
// Ported from BroncBotz 3481 FRC2025 (comp branch), which used FRC 6328's 2025 field constants.
// Only the reef branch positions are still used.

package first.robot.util;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;

/**
 * Contains various field dimensions and useful reference points. All units are in meters and poses
 * have a blue alliance origin.
 */
public final class FieldConstants {
    public static final double fieldLength = Units.inchesToMeters(690.876);
    public static final double fieldWidth = Units.inchesToMeters(317);

    public enum ReefHeight {
        L4(Units.inchesToMeters(54), 44),
        L3(Units.inchesToMeters(48), 14),
        L2(Units.inchesToMeters(42), -14),
        L1(Units.inchesToMeters(42), -56);

        public final double height;
        public final double pitch;

        ReefHeight(double height, double pitch) {
            this.height = height;
            this.pitch = pitch; // in degrees
        }
    }

    public static final class Reef {
        public static final Translation2d center =
            new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.501));

        // Starting at the left branch facing the driver station, then clockwise.
        public static final List<Map<ReefHeight, Pose3d>> branchPositions = new ArrayList<>(12);

        static {
            for (int face = 0; face < 6; face++) {
                Map<ReefHeight, Pose3d> fillRight = new HashMap<>();
                Map<ReefHeight, Pose3d> fillLeft = new HashMap<>();
                Pose2d poseDirection = new Pose2d(center, Rotation2d.fromDegrees(180 - (60 * face)));
                double adjustX = Units.inchesToMeters(30.738);
                double adjustY = Units.inchesToMeters(6.469);
                for (var level : ReefHeight.values()) {
                    fillRight.put(level, branchPose(poseDirection, adjustX, adjustY, level));
                    fillLeft.put(level, branchPose(poseDirection, adjustX, -adjustY, level));
                }
                branchPositions.add(fillLeft);
                branchPositions.add(fillRight);
            }
        }

        private static Pose3d branchPose(Pose2d poseDirection, double adjustX, double adjustY, ReefHeight level) {
            Pose2d branch = poseDirection.transformBy(new Transform2d(adjustX, adjustY, Rotation2d.ZERO));
            return new Pose3d(
                new Translation3d(branch.getX(), branch.getY(), level.height),
                new Rotation3d(0, Units.degreesToRadians(level.pitch), poseDirection.getRotation().getRadians()));
        }
    }

    private FieldConstants() {
    }
}
