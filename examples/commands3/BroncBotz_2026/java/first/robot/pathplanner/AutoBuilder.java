// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.pathplanner;

import java.io.File;
import java.util.Arrays;
import java.util.List;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.system.Filesystem;

/**
 * Stand-in for PathPlannerLib's {@code AutoBuilder}, which is built on commands v2 and cannot be
 * used alongside commands v3. Commands v3 support for PathPlanner is not finished yet, so every auto
 * reports an error and does nothing. Teleop is unaffected. The autos in
 * {@code deploy/pathplanner/autos} and the named commands and event triggers registered in
 * {@code Robot} are kept for when it is.
 */
public final class AutoBuilder {
    private AutoBuilder() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Names of every auto in the deploy directory, like PathPlannerLib's
     * {@code AutoBuilder.getAllAutoNames()}.
     *
     * @return Auto names in alphabetical order, or none if the directory cannot be read.
     */
    public static List<String> getAllAutoNames() {
        final File[] files = new File(Filesystem.getDeployDirectory(), "pathplanner/autos")
            .listFiles((dir, name) -> name.endsWith(".auto"));
        if (files == null) {
            return List.of();
        }
        return Arrays.stream(files)
            .map(file -> file.getName().substring(0, file.getName().length() - ".auto".length()))
            .sorted()
            .toList();
    }

    /**
     * Build the command for an auto. Not available yet: reports an error and does nothing.
     *
     * @param autoName Name of the auto in the PathPlanner GUI.
     * @param mirror   Mirror the auto across the field's long centerline, to run it on the other side.
     * @return A command that does nothing.
     */
    public static Command buildAuto(String autoName, boolean mirror) {
        DriverStationErrors.reportError("PathPlanner is not available with commands v3 yet; auto " + autoName
            + " does nothing", false);
        return Command.noRequirements(coroutine -> {}).named(autoName + " (PathPlanner unavailable)");
    }
}
