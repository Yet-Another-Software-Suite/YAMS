// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.pathplanner;

import org.wpilib.command3.Command;

/**
 * Stand-in for PathPlannerLib's {@code EventTrigger}: commands that start when a followed path passes
 * an event marker with a given name. Paths are not followed until commands v3 support for PathPlanner
 * is finished (see {@link AutoBuilder}), so registered commands never start.
 */
public final class EventTriggers {
    private EventTriggers() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Start a command whenever a followed path passes a marker with this name. Not available yet.
     *
     * @param name    Marker name used in the PathPlanner GUI.
     * @param command Command to start.
     */
    public static void onTrue(String name, Command command) {
    }
}
