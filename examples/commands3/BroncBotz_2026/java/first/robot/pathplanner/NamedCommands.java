// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot.pathplanner;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;

/**
 * Commands that PathPlanner autos and event markers can run by name, like PathPlannerLib's
 * {@code NamedCommands}. Each name is registered with a factory, so every place an auto uses the
 * name gets its own command.
 */
public final class NamedCommands {
    private static final Map<String, Supplier<Command>> namedCommands = new HashMap<>();

    private NamedCommands() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Register a named command. Register every named command before building the autos that use it.
     *
     * @param name    Name used in the PathPlanner GUI.
     * @param command Builds the command each time an auto or event marker uses it.
     */
    public static void registerCommand(String name, Supplier<Command> command) {
        namedCommands.put(name, command);
    }

    /**
     * Build the command registered under the given name. An unknown name reports a warning and does
     * nothing, as PathPlannerLib did.
     *
     * @param name Name used in the PathPlanner GUI.
     * @return A new command.
     */
    public static Command getCommand(String name) {
        final Supplier<Command> command = namedCommands.get(name);
        if (command == null) {
            DriverStationErrors.reportWarning("PathPlanner attempted to create a command '" + name
                + "' that has not been registered with NamedCommands.registerCommand", false);
            return Command.noRequirements(coroutine -> {}).named("Unregistered " + name);
        }
        return command.get();
    }
}
