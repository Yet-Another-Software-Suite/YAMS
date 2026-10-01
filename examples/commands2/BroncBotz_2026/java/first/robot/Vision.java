// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;

import com.limelightvision.FiducialTarget;
import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import java.util.Optional;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;

/**
 * MegaTag1 pose estimates from one Limelight, read through LimelightLib. The robot has two: one on
 * the drivetrain facing backwards and one on the (fixed) turret.
 */
public class Vision {
    // AprilTags each camera may use; the rest are filtered out on the camera.
    private static final int[] kDrivetrainTags = {
        1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };
    private static final int[] kTurretTags = {
        1, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 31, 32
    };

    // Estimates are only trusted with several close, unambiguous tags.
    private static final double kMaxAmbiguity = 0.2;
    private static final int kMinTags = 2;
    private static final double kMaxTagDistanceMeters = Feet.of(18).in(Meters);

    private final Limelight limelight;
    private final DoublePublisher ambiguityPublisher;
    private final StructPublisher<Pose2d> posePublisher;
    private double lastTimestampSeconds = 0;

    /**
     * @param name       Limelight name.
     * @param cameraPose Camera pose in robot space.
     * @param tagFilter  AprilTag IDs to use.
     */
    private Vision(String name, Pose3d cameraPose, int[] tagFilter) {
        limelight = new Limelight(name);
        limelight.setPipelineIndex(0);
        limelight.setCameraPose_RobotSpaceOverride(cameraPose, false);
        limelight.setFiducialIDFiltersOverride(tagFilter);
        Limelight.flushNT();
        final var table = NetworkTableInstance.getDefault().getTable("Vision/" + name);
        ambiguityPublisher = table.getDoubleTopic("Ambiguity").publish();
        posePublisher = table.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    /** The drivetrain Limelight: 12 in back and 12 in right of center, 10 in up, pitched 45° and facing backwards. */
    public static Vision drivetrainCamera() {
        return new Vision("limelight",
            new Pose3d(Inches.of(-12), Inches.of(-12), Inches.of(10), new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(180))),
            kDrivetrainTags);
    }

    /** The turret Limelight: centered, 20.5 in up, pitched 45° and facing forwards. */
    public static Vision turretCamera() {
        return new Vision("limelight-turret",
            new Pose3d(Inches.of(0), Inches.of(0), Inches.of(20.5), new Rotation3d(Degrees.of(0), Degrees.of(45), Degrees.of(0))),
            kTurretTags);
    }

    /**
     * Get a new, trusted pose estimate.
     *
     * @param currentPose The robot's current pose, sent to the camera as its orientation.
     * @return The estimate, or empty if there is no new estimate or it is not trusted.
     */
    public Optional<PoseEstimate> getMeasurement(Pose2d currentPose) {
        limelight.setRobotOrientation(currentPose.getRotation().getDegrees(), true);
        final PoseEstimate estimate = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
        if (estimate == null || !estimate.isValid() || estimate.timestampSeconds == lastTimestampSeconds) {
            return Optional.empty();
        }
        final double ambiguity = averageAmbiguity(estimate);
        ambiguityPublisher.set(ambiguity);
        if (ambiguity >= kMaxAmbiguity || estimate.fieldedTagCount < kMinTags || estimate.avgTagDistanceMeters >= kMaxTagDistanceMeters) {
            return Optional.empty();
        }
        lastTimestampSeconds = estimate.timestampSeconds;
        posePublisher.set(estimate.pose);
        return Optional.of(estimate);
    }

    private static double averageAmbiguity(PoseEstimate estimate) {
        final FiducialTarget[] fiducials = estimate.getFieldedFiducials();
        if (fiducials == null || fiducials.length == 0) {
            return 1;
        }
        double sum = 0;
        for (FiducialTarget fiducial : fiducials) {
            sum += fiducial.ambiguity;
        }
        return sum / fiducials.length;
    }
}
