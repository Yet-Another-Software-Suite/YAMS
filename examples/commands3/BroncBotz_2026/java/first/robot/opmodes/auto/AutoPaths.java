// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.opmodes.auto;

import java.util.List;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * The paths the autos drive, taken from the original PathPlanner paths. Poses are blue alliance origin, in meters and
 * degrees. Each path is its anchor points; the robot drives to each in turn with drive to pose, so the curves between
 * them are straight lines.
 *
 * <p>PathPlanner turns the robot through rotation targets placed between anchor points. Here each waypoint takes the
 * heading of the nearest rotation target, the start rotation, or the end rotation, and the last waypoint takes the end
 * rotation. PathPlanner constraint zones, which slowed the robot along parts of a path, are not kept.
 */
final class AutoPaths {
    private AutoPaths() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /** Commands started alongside a path, from the PathPlanner event markers. */
    enum Marker {
        /** Run the intake in, until {@link #INTAKE_STOP} or a shot takes over the agitator. */
        INTAKE_START,
        /** Stop the intake rollers, keeping the agitator stirring. */
        INTAKE_STOP
    }

    /**
     * An event marker. PathPlanner starts the command at a fraction of the way between two anchor points; here it starts
     * once the robot reaches the anchor point before it.
     *
     * @param waypoint Index of the waypoint the command starts at, 0 for the start of the path.
     * @param marker   Command to start.
     */
    record MarkerAt(int waypoint, Marker marker) {}

    /**
     * A path.
     *
     * @param name      Name of the original PathPlanner path.
     * @param waypoints Waypoints, blue alliance origin. The first is where the path starts.
     * @param markers   Event markers.
     */
    record AutoPath(String name, List<Pose2d> waypoints, List<MarkerAt> markers) {
        /**
         * Where the path starts.
         *
         * @return Starting pose, blue alliance origin.
         */
        Pose2d start() {
            return waypoints.get(0);
        }
    }

    private static Pose2d pose(double x, double y, double headingDegrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(headingDegrees));
    }

    /** PathPlanner path "Auto 2 Preload Path". */
    static final AutoPath AUTO_2_PRELOAD_PATH = new AutoPath(
            "Auto 2 Preload Path",
            List.of(pose(3.534, 4.054, 0),
                    pose(2.141, 4.054, 0)),
            List.of());

    /** PathPlanner path "Auto One Advanced Path 1". */
    static final AutoPath AUTO_ONE_ADVANCED_PATH_1 = new AutoPath(
            "Auto One Advanced Path 1",
            List.of(pose(3.549, 7.425, 0),
                    pose(5.494, 7.425, 0),
                    pose(7.33, 7.226, -90),
                    pose(7.724, 5.193, -90),
                    pose(6.849, 6.078, -90),
                    pose(5.243, 7.357, 0),
                    pose(3.538, 7.423, 0),
                    pose(2.5, 6.308, -47.726)),
            List.of(new MarkerAt(0, Marker.INTAKE_START), new MarkerAt(3, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto One Advanced Path 2". */
    static final AutoPath AUTO_ONE_ADVANCED_PATH_2 = new AutoPath(
            "Auto One Advanced Path 2",
            List.of(pose(2.429, 6.363, -50.015),
                    pose(6.583, 7.045, -90),
                    pose(6.777, 4.887, -90),
                    pose(6.777, 7.223, 0),
                    pose(3.56, 7.426, 0),
                    pose(2.429, 6.363, -47.291)),
            List.of(new MarkerAt(0, Marker.INTAKE_START), new MarkerAt(2, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto One Advanced Path Follow". */
    static final AutoPath AUTO_ONE_ADVANCED_PATH_FOLLOW = new AutoPath(
            "Auto One Advanced Path Follow",
            List.of(pose(2.429, 6.509, -45),
                    pose(3.565, 7.434, 0)),
            List.of());

    /** PathPlanner path "Auto One Path One". */
    static final AutoPath AUTO_ONE_PATH_ONE = new AutoPath(
            "Auto One Path One",
            List.of(pose(3.56, 7.375, 0),
                    pose(7.08, 7.095, -90),
                    pose(7.584, 4.857, -90),
                    pose(7.3, 7.211, 0),
                    pose(2.422, 7.16, 0),
                    pose(2.085, 6.254, -45)),
            List.of(new MarkerAt(1, Marker.INTAKE_START)));

    /** PathPlanner path "Auto Three Advanced Path 1 Mirror". */
    static final AutoPath AUTO_THREE_ADVANCED_PATH_1_MIRROR = new AutoPath(
            "Auto Three Advanced Path 1 Mirror",
            List.of(pose(3.56, 7.406, 0),
                    pose(5.494, 7.406, 0),
                    pose(7.429, 7.04, -90),
                    pose(7.341, 6.319, -90),
                    pose(6.642, 6.811, -90),
                    pose(5.757, 7.401, 0),
                    pose(3.549, 7.406, 0),
                    pose(3, 7, -54.69)),
            List.of(new MarkerAt(0, Marker.INTAKE_START), new MarkerAt(3, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto Three Advanced Path 1". */
    static final AutoPath AUTO_THREE_ADVANCED_PATH_1 = new AutoPath(
            "Auto Three Advanced Path 1",
            List.of(pose(3.549, 0.613, 0),
                    pose(6.762, 0.757, 0),
                    pose(7.56, 1.784, 90),
                    pose(6.991, 2.134, 90),
                    pose(6.73, 1.454, 0),
                    pose(5.9, 0.613, 0),
                    pose(3.3, 0.613, 0),
                    pose(2.664, 0.789, 54.69)),
            List.of(new MarkerAt(0, Marker.INTAKE_START), new MarkerAt(3, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto Three Advanced Path 2 In". */
    static final AutoPath AUTO_THREE_ADVANCED_PATH_2_IN = new AutoPath(
            "Auto Three Advanced Path 2 In",
            List.of(pose(2.664, 0.789, 54.69),
                    pose(3.189, 0.613, 0),
                    pose(5.582, 0.613, 0),
                    pose(6.64, 1.674, 90),
                    pose(7.61, 2.062, 90),
                    pose(6.368, 0.613, 0),
                    pose(4.582, 0.613, -0.033)),
            List.of(new MarkerAt(2, Marker.INTAKE_START), new MarkerAt(4, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto Three Advanced Path 2 Mirror". */
    static final AutoPath AUTO_THREE_ADVANCED_PATH_2_MIRROR = new AutoPath(
            "Auto Three Advanced Path 2 Mirror",
            List.of(pose(3, 7, -54.69),
                    pose(3.5, 7.406, 0),
                    pose(5.494, 7.406, 0),
                    pose(6.412, 6.876, -90),
                    pose(6.915, 5.423, -90),
                    pose(7.254, 6.985, -90),
                    pose(5.6, 7.406, 0),
                    pose(3.549, 7.406, 0),
                    pose(3.3, 7.306, -54.69)),
            List.of(new MarkerAt(1, Marker.INTAKE_START), new MarkerAt(4, Marker.INTAKE_STOP)));

    /** PathPlanner path "Auto Three Path Three". */
    static final AutoPath AUTO_THREE_PATH_THREE = new AutoPath(
            "Auto Three Path Three",
            List.of(pose(3.56, 0.639, 0),
                    pose(6.666, 0.742, 0),
                    pose(7.235, 1.053, 90),
                    pose(7.649, 2.618, 90),
                    pose(7.52, 2.411, 90),
                    pose(5.968, 0.639, 0),
                    pose(3.56, 0.639, 0),
                    pose(2.228, 1.273, 47.816)),
            List.of(new MarkerAt(0, Marker.INTAKE_START)));

    /** PathPlanner path "Auto Two Path One". */
    static final AutoPath AUTO_TWO_PATH_ONE = new AutoPath(
            "Auto Two Path One",
            List.of(pose(3.547, 3.977, 180),
                    pose(2.293, 5.663, 180),
                    pose(1.162, 5.8, 180),
                    pose(1.175, 6.1, 180)),
            List.of(new MarkerAt(1, Marker.INTAKE_START)));

    /** PathPlanner path "Auto Two Path Two". */
    static final AutoPath AUTO_TWO_PATH_TWO = new AutoPath(
            "Auto Two Path Two",
            List.of(pose(1.211, 6.1, 179.892),
                    pose(1.801, 4.92, -38.956)),
            List.of(new MarkerAt(0, Marker.INTAKE_STOP)));
}
