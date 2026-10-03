// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.mechanisms;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;

import com.limelightvision.Limelight;
import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import yams.core.mechanisms.swerve.SwerveDrive;

/**
 * Limelight MegaTag1 pose estimates fused into the drivetrain's pose estimator, as the original did
 * from its swerve subsystem with YALL. Read through LimelightLib.
 */
class Vision {
    // Front left corner, 10.5 in up, turned 45 degrees toward the front left.
    private static final Pose3d kCameraPose = new Pose3d(
        Inches.of(12), Inches.of(12), Inches.of(10.5), new Rotation3d(Degrees.zero(), Degrees.zero(), Degrees.of(45)));
    // Reef tags only.
    private static final int[] kTagFilter = {17, 18, 19, 20, 21, 22, 6, 7, 8, 9, 10, 11};
    private static final Matrix<N3, N1> kStandardDeviations = VecBuilder.fill(0.05, 0.05, 0.022);
    // Estimates farther than this from odometry are only accepted after kMaxRejections in a row, so
    // one bad frame cannot move the robot but a wrong starting pose still gets corrected.
    private static final double kMaxJumpMeters = 0.5;
    private static final int kMaxRejections = 10;

    private final SwerveDrive drive;
    private final Limelight limelight = new Limelight("limelight", kCameraPose);
    private int rejections = 0;

    Vision(SwerveDrive drive) {
        this.drive = drive;
        limelight.setFiducialIDFiltersOverride(kTagFilter);
    }

    void update() {
        limelight.setRobotOrientation(drive.getPose().getRotation().getDegrees(), true);
        final PoseEstimate estimate = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
        if (estimate == null || !estimate.isValid()) {
            return;
        }
        final double distance = estimate.pose.getTranslation().getDistance(drive.getPose().getTranslation());
        if (distance < kMaxJumpMeters || rejections > kMaxRejections) {
            rejections = 0;
            drive.addVisionMeasurement(estimate.pose, estimate.timestampSeconds, kStandardDeviations);
        } else {
            rejections++;
        }
    }
}
