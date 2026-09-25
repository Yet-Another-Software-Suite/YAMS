// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot.subsystems;

import com.limelightvision.PoseEstimate;
import com.limelightvision.PoseEstimateType;
import java.util.Optional;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StructPublisher;

/** MegaTag pose estimates from a Limelight, read through LimelightLib. */
public class Limelight extends SubsystemBase {
    private final com.limelightvision.Limelight limelight;
    private final NetworkTable telemetryTable;
    private final StructPublisher<Pose2d> posePublisher;

    public Limelight(String name) {
        this.limelight = new com.limelightvision.Limelight(name);
        this.telemetryTable = NetworkTableInstance.getDefault().getTable("WCPCC_Limelight/" + name);
        this.posePublisher = telemetryTable.getStructTopic("Estimated Robot Pose", Pose2d.struct).publish();
    }

    public Optional<Measurement> getMeasurement(Pose2d currentRobotPose) {
        // Only this camera is updated each loop, so flush the orientation right away.
        limelight.setRobotOrientation(currentRobotPose.getRotation().getDegrees(), true);

        final PoseEstimate poseEstimate_MegaTag1 = limelight.getPoseEstimate(PoseEstimateType.MT1_WPIBLUE);
        final PoseEstimate poseEstimate_MegaTag2 = limelight.getPoseEstimate(PoseEstimateType.MT2_WPIBLUE);

        // isValid() requires at least one fielded tag and no rejected checks.
        if (
            poseEstimate_MegaTag1 == null
                || poseEstimate_MegaTag2 == null
                || !poseEstimate_MegaTag1.isValid()
                || !poseEstimate_MegaTag2.isValid()
        ) {
            return Optional.empty();
        }

        // Combine the readings from MegaTag1 and MegaTag2:
        // 1. Use the more stable position from MegaTag2
        // 2. Use the rotation from MegaTag1 (with low confidence) to counteract gyro drift
        poseEstimate_MegaTag2.pose = new Pose2d(
            poseEstimate_MegaTag2.pose.getTranslation(),
            poseEstimate_MegaTag1.pose.getRotation()
        );
        final Matrix<N3, N1> standardDeviations = VecBuilder.fill(0.1, 0.1, 10.0);

        posePublisher.set(poseEstimate_MegaTag2.pose);

        return Optional.of(new Measurement(poseEstimate_MegaTag2, standardDeviations));
    }

    public static class Measurement {
        public final PoseEstimate poseEstimate;
        public final Matrix<N3, N1> standardDeviations;

        public Measurement(PoseEstimate poseEstimate, Matrix<N3, N1> standardDeviations) {
            this.poseEstimate = poseEstimate;
            this.standardDeviations = standardDeviations;
        }
    }
}
