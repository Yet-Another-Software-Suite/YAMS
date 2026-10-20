// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.Subsystem;
import org.wpilib.tunable.Tunables;
import yams.core.telemetry.NetworkTablesBackends;

/**
 * Registry for WPILib {@link org.wpilib.command2.Command} instances associated with a
 * {@link yams.core.motorcontrollers.SmartMotorController}. Mechanisms register setpoint commands
 * here so the scheduler can manage them.
 */
public class SmartMotorControllerCommandRegistry {
  /**
   * HashMap with the Subsystem name as the key and the shared command which runs all runnables
   * added to the subsystem.
   */
  private static Map<String, Command>        commands         = new HashMap<>();
  /**
   * HashMap with the Subsystem name as the key and a list of runnables to be added to the shared
   * command.
   */
  private static Map<String, List<Runnable>> commandCallbacks = new HashMap<>();
  /**
   * HashMap with the Subsystem name as the key and a list of runnables run when the shared command
   * ends.
   */
  private static Map<String, List<Runnable>> commandEndCallbacks = new HashMap<>();
  /**
   * HashMap tracking which subsystem instance owns each command key.
   * Used to detect when two different subsystem instances share the same name.
   */
  private static Map<String, Subsystem>      commandOwners    = new HashMap<>();

  /** Static registry, not instantiable. */
  private SmartMotorControllerCommandRegistry() {}

  /**
   * Create the {@link Command} and publish it to NetworkTables.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param subsystem Subsystem to create the command for.
   */
  private static void addCommandToNT(String cmdName, Subsystem subsystem) {
    var key = subsystem.getName() + "/" + cmdName;
    Command cmd = Commands.run(() -> {
      for (var callback : commandCallbacks.get(key)) {
        callback.run();
      }
    }, subsystem).finallyDo(() -> runEndCallbacks(key)).withName(cmdName);
    commands.put(key, cmd);
    NetworkTablesBackends.ensureTuningTunableBackend();
    Tunables.publish("Tuning/" + key, cmd);
  }

  /**
   * Add a command to the registry.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param subsystem Subsystem to create the command for.
   * @param callback  Runnable to be added to the shared command.
   * @throws IllegalStateException if a different {@link Subsystem} instance with the same name has
   *                               already registered a command under {@code cmdName}.
   */
  public static void addCommand(String cmdName, Subsystem subsystem, Runnable callback) {
    addCommand(cmdName, subsystem, callback, () -> {});
  }

  /**
   * Add a command to the registry, with a {@link Runnable} run when the shared command ends.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param subsystem Subsystem to create the command for.
   * @param callback  Runnable to be added to the shared command.
   * @param onEnd     Runnable run when the shared command ends.
   * @throws IllegalStateException if a different {@link Subsystem} instance with the same name has
   *                               already registered a command under {@code cmdName}.
   */
  public static void addCommand(String cmdName, Subsystem subsystem, Runnable callback, Runnable onEnd) {
    var key = subsystem.getName() + "/" + cmdName;
    var owner = commandOwners.get(key);
    if (owner != null && owner != subsystem) {
      throw new IllegalStateException("SmartMotorControllerCommandRegistry: subsystem name conflict \"" + subsystem
          .getName() + "\" is already registered by a different subsystem instance. " + ("Use unique subsystem names for each subsystem instance (e.g. \"LeftTurret\", " + "\"RightTurret\")."));
    }
    commandOwners.put(key, subsystem);
    commandCallbacks.computeIfAbsent(key, k -> new ArrayList<>()).add(callback);
    commandEndCallbacks.computeIfAbsent(key, k -> new ArrayList<>()).add(onEnd);
    // Create Command and publish it to NT
    if (!commandExists(cmdName, subsystem)) {
      addCommandToNT(cmdName, subsystem);
    }
  }

  /**
   * Run the callbacks registered to run when the shared command ends.
   *
   * @param key Command key, the Subsystem name and the command name.
   */
  private static void runEndCallbacks(String key) {
    for (var onEnd : commandEndCallbacks.getOrDefault(key, List.of())) {
      onEnd.run();
    }
  }

  /**
   * Get a registered command, such as the "Live Tuning" command, to schedule it from code.
   *
   * @param cmdName   Command name.
   * @param subsystem Subsystem the command was registered for.
   * @return The command, or empty if none is registered under {@code cmdName}.
   */
  public static Optional<Command> getCommand(String cmdName, Subsystem subsystem) {
    return Optional.ofNullable(commands.get(subsystem.getName() + "/" + cmdName));
  }

  /**
   * Check if a command exists.
   *
   * @param cmdName   Command name.
   * @param subsystem Subsystem.
   * @return True if command exists.
   */
  public static boolean commandExists(String cmdName, Subsystem subsystem) {
    var key = subsystem.getName() + "/" + cmdName;
    return commands.containsKey(key);
  }

  /**
   * Remove all commands registered for the given subsystem instance.
   *
   * <p>Call this when a subsystem is being torn down (e.g. between tests) so that a
   * new instance with the same name can register without triggering the conflict check.
   *
   * @param subsystem Subsystem whose commands should be removed.
   */
  public static void removeCommands(Subsystem subsystem) {
    commandOwners.entrySet().removeIf(e -> {
      if (e.getValue() == subsystem) {
        commandCallbacks.remove(e.getKey());
        commandEndCallbacks.remove(e.getKey());
        commands.remove(e.getKey());
        Tunables.remove("Tuning/" + e.getKey());
        return true;
      }
      return false;
    });
  }
}
