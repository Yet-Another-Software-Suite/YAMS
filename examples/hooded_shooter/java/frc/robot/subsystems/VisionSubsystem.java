// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import java.util.Comparator;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import org.wpilib.vision.apriltag.AprilTagFieldLayout;
import org.wpilib.vision.apriltag.AprilTagFields;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.framework.RobotBase;
import org.wpilib.command2.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera;
    VisionSystemSim visionSim;
    AprilTagFieldLayout tagLayout;

    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim;

    /** True if photonvision JNI loaded successfully; false means vision is unavailable. */
    private boolean photonAvailable = false;

    public VisionSubsystem() {
        try {
            camera = new PhotonCamera("HubOrientedCameraMountedOnTurret");
            tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
            photonAvailable = true;
        } catch (UnsatisfiedLinkError | Exception e) {
            System.err.println("[VisionSubsystem] PhotonVision unavailable (JNI load failed): " + e.getMessage());
            return;
        }

        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");

            // A 640 x 480 camera with a 100 degree diagonal FOV.
            cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in
            // pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop
            // rate).
            cameraProp.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            cameraSim = new PhotonCameraSim(camera, cameraProp);

            Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
            // and pitched 15 degrees up.
            Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
            Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

            // Add this camera to the vision system simulation with the given
            // robot-to-camera transform.
            visionSim.addCamera(cameraSim, robotToCamera);
        }
    }

    @Override
    public void periodic() {
        if (!photonAvailable) return;

        if (RobotBase.isSimulation() && visionSim != null) {
            visionSim.update(new Pose2d());
        }

        camera.getAllUnreadResults().forEach((result) -> {
            if (result.getBestTarget().getFiducialId() > 0) {
            }
        });
    }

    public Optional<PhotonTrackedTarget> getClosestTag() {
        if (!photonAvailable || camera == null) return Optional.empty();
        for(var results : camera.getAllUnreadResults()
        ){
            return results.hasTargets()
                ? results.getTargets().stream()
                        .filter(t -> t.getFiducialId() > 0) // AprilTags only
                        .min(Comparator.comparingDouble(
                                t -> t.getBestCameraToTarget().getTranslation().getNorm()))
                : Optional.empty();
        }
        return Optional.empty();
    }
}