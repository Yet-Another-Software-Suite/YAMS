// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.telemetry;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.tunable.Tunables;
import yams.core.telemetry.NetworkTablesBackends;

/**
 * Registry for {@link org.wpilib.command3.Command} instances associated with a
 * {@link yams.core.motorcontrollers.SmartMotorController}. Mechanisms register setpoint commands
 * here so the scheduler can manage them.
 */
public class SmartMotorControllerCommandRegistry {
  /**
   * HashMap with the Mechanism name as the key and the shared command which runs all runnables
   * added to the mechanism.
   */
  private static Map<String, Command>        commands         = new HashMap<>();
  /**
   * HashMap with the Mechanism name as the key and a list of runnables to be added to the shared
   * command.
   */
  private static Map<String, List<Runnable>> commandCallbacks = new HashMap<>();
  /**
   * HashMap tracking which mechanism instance owns each command key.
   * Used to detect when two different mechanism instances share the same name.
   */
  private static Map<String, Mechanism>      commandOwners    = new HashMap<>();

  /**
   * Create the {@link Command} and publish it to NetworkTables.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param mechanism Mechanism to create the command for.
   */
  private static void addCommandToNT(String cmdName, Mechanism mechanism) {
    var key = mechanism.getName() + "/" + cmdName;
    Command cmd = mechanism.run(coroutine -> {
      while (true) {
        for (var callback : commandCallbacks.get(key)) {
          callback.run();
        }
        coroutine.yield();
      }
    }).named(cmdName);
    commands.put(key, cmd);
    NetworkTablesBackends.ensureTuningTunableBackend();
    Tunables.publish("Tuning/" + key, new CommandTunable(cmd));
  }

  /**
   * Add a command to the registry.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param mechanism Mechanism to create the command for.
   * @param callback  Runnable to be added to the shared command.
   */
  public static void addCommand(String cmdName, Mechanism mechanism, Runnable callback) {
    var key = mechanism.getName() + "/" + cmdName;
    var owner = commandOwners.get(key);
    if (owner != null && owner != mechanism) {
      throw new IllegalStateException("SmartMotorControllerCommandRegistry: mechanism name conflict \"" + mechanism
          .getName() + "\" is already registered by a different mechanism instance. " + ("Use unique mechanism names for each mechanism instance (e.g. \"LeftTurret\", " + "\"RightTurret\")."));
    }
    commandOwners.put(key, mechanism);
    commandCallbacks.computeIfAbsent(key, k -> new ArrayList<>()).add(callback);
    // Create Command and publish it to NT
    if (!commandExists(cmdName, mechanism)) {
      addCommandToNT(cmdName, mechanism);
    }
  }

  /**
   * Check if a command exists.
   *
   * @param cmdName   Command name.
   * @param mechanism Mechanism.
   * @return True if command exists.
   */
  public static boolean commandExists(String cmdName, Mechanism mechanism) {
    var key = mechanism.getName() + "/" + cmdName;
    return commands.containsKey(key);
  }

  /**
   * Remove all commands registered for the given mechanism instance.
   *
   * <p>Call this when a mechanism is being torn down (e.g. between tests) so that a
   * new instance with the same name can register without triggering the conflict check.
   *
   * @param mechanism Mechanism whose commands should be removed.
   */
  public static void removeCommands(Mechanism mechanism) {
    commandOwners.entrySet().removeIf(e -> {
      if (e.getValue() == mechanism) {
        commandCallbacks.remove(e.getKey());
        commands.remove(e.getKey());
        Tunables.remove("Tuning/" + e.getKey());
        return true;
      }
      return false;
    });
  }
}
