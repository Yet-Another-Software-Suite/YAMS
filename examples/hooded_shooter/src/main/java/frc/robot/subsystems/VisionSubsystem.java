package frc.robot.subsystems;

import java.util.Comparator;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
    PhotonCamera camera = new PhotonCamera("HubOrientedCameraMountedOnTurret");
    VisionSystemSim visionSim = new VisionSystemSim("main");
    AprilTagFieldLayout tagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    SimCameraProperties cameraProp = new SimCameraProperties();
    PhotonCameraSim cameraSim;

    public VisionSubsystem() {
        if (RobotBase.isSimulation()) {
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
        if (RobotBase.isSimulation()) {
            visionSim.update(new Pose2d());
        }

        camera.getAllUnreadResults().forEach((result) -> {
            if (result.getBestTarget().getFiducialId() > 0) {

            }
        });
    }

    public Optional<PhotonTrackedTarget> getClosestTag() {
        return camera.getLatestResult().hasTargets()
                ? camera.getLatestResult().getTargets().stream()
                        .filter(t -> t.getFiducialId() > 0) // AprilTags only
                        .min(Comparator.comparingDouble(
                                t -> t.getBestCameraToTarget().getTranslation().getNorm()))
                : Optional.empty();
    }
}
