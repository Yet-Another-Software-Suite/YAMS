// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.events.OneShotTriggerEvent;
import com.pathplanner.lib.events.TriggerEvent;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import java.io.File;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Checks the event timing the Commands v3 port uses. Commands v3 cannot let PathPlanner create its
 * events (they need Commands v2), so it generates each trajectory from the path without its event
 * markers and times each marker itself: the first trajectory state whose waypoint relative position
 * is closest to the marker's. This test runs PathPlanner's own event timing on every autonomous path
 * and checks the two agree.
 */
class PathEventTimingTest {
    private record TimedEvent(String name, double timeSeconds) {
    }

    @BeforeAll
    static void initializeHal() {
        // PathPlanner's RobotConfig creates an Alert, which needs the HAL.
        assertTrue(HAL.initialize());
    }

    @Test
    void markerTimingMatchesPathPlanner() throws Exception {
        final RobotConfig config = RobotConfig.fromGUISettings();
        final File[] pathFiles = new File("src/main/deploy/pathplanner/paths").listFiles((dir, name) -> name.endsWith(".path"));
        assertTrue(pathFiles != null && pathFiles.length > 0, "No PathPlanner paths found");
        for (File file : pathFiles) {
            final String name = file.getName().replace(".path", "");
            final PathPlannerPath path = PathPlannerPath.fromPathFile(name);
            final ChassisVelocities startSpeeds = new ChassisVelocities();

            final PathPlannerTrajectory withMarkers = path.generateTrajectory(startSpeeds, path.getInitialHeading(), config);
            final List<TimedEvent> expected = new ArrayList<>();
            for (Event event : withMarkers.getEvents()) {
                if (event instanceof OneShotTriggerEvent oneShot) {
                    expected.add(new TimedEvent(oneShot.getEventName(), event.getTimestampSeconds()));
                } else if (event instanceof TriggerEvent trigger && trigger.getValue()) {
                    expected.add(new TimedEvent(trigger.getEventName(), event.getTimestampSeconds()));
                }
            }

            final PathPlannerPath stripped = new PathPlannerPath(path.getWaypoints(), path.getRotationTargets(),
                path.getPointTowardsZones(), path.getConstraintZones(), List.of(), path.getGlobalConstraints(),
                path.getIdealStartingState(), path.getGoalEndState(), path.isReversed());
            final PathPlannerTrajectory withoutMarkers = stripped.generateTrajectory(startSpeeds, path.getInitialHeading(), config);
            assertTrue(withoutMarkers.getEvents().isEmpty(), name + ": stripped path still has events");
            assertEquals(withMarkers.getTotalTimeSeconds(), withoutMarkers.getTotalTimeSeconds(), 1e-9, name + ": trajectories differ");

            final List<TimedEvent> actual = timeMarkers(stripped, withoutMarkers, path.getEventMarkers());
            assertEquals(expected.size(), actual.size(), name + ": event count");
            for (int i = 0; i < expected.size(); i++) {
                assertEquals(expected.get(i).name(), actual.get(i).name(), name + ": event order");
                assertEquals(expected.get(i).timeSeconds(), actual.get(i).timeSeconds(), 1e-9, name + ": " + expected.get(i).name() + " time");
            }
        }
    }

    /** The same timing as {@code PathEvents.time(...)} in the Commands v3 port. */
    private static List<TimedEvent> timeMarkers(PathPlannerPath path, PathPlannerTrajectory trajectory, List<EventMarker> markers) {
        final List<EventMarker> sorted = new ArrayList<>(markers);
        sorted.sort(Comparator.comparingDouble(EventMarker::position));
        final List<PathPoint> points = path.getAllPathPoints();
        final List<TimedEvent> timed = new ArrayList<>();
        int next = 0;
        for (int i = 1; i < points.size() && next < sorted.size(); i++) {
            final double previousPosition = points.get(i - 1).waypointRelativePos;
            final double position = points.get(i).waypointRelativePos;
            while (next < sorted.size()
                && Math.abs(sorted.get(next).position() - previousPosition) <= Math.abs(sorted.get(next).position() - position)) {
                timed.add(new TimedEvent(sorted.get(next).triggerName(), trajectory.getState(i - 1).timeSeconds));
                next++;
            }
        }
        for (; next < sorted.size(); next++) {
            timed.add(new TimedEvent(sorted.get(next).triggerName(), trajectory.getEndState().timeSeconds));
        }
        return timed;
    }
}
