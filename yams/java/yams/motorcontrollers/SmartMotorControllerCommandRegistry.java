package yams.motorcontrollers;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Registry for SmartMotorController commands.
 */
public class SmartMotorControllerCommandRegistry
{

  /**
   * HashMap with the Subsystem name as the key and the shared command which runs all runnables added to the subsystem.
   */
  private static Map<String, Command>        commands         = new HashMap<>();
  /**
   * HashMap with the Subsystem name as the key and a list of runnables to be added to the shared command.
   */
  private static Map<String, List<Runnable>> commandCallbacks = new HashMap<>();

  /**
   * Create the {@link Command} and publish it to NetworkTables.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param subsystem Subsystem to create the command for.
   */
  private static void addCommandToNT(String cmdName, Subsystem subsystem)
  {
    var key = subsystem.getName() + "/" + cmdName;
    Command cmd = Commands.run(() -> {
      for (var callback : commandCallbacks.get(key))
      {
        callback.run();
      }
    }, subsystem).withName(cmdName);
    commands.put(key, cmd);
    SmartDashboard.putData("Mechanisms/Commands/" + key, cmd);
  }

  /**
   * Add a command to the registry.
   *
   * @param cmdName   Command name to publish to NetworkTables.
   * @param subsystem Subsystem to create the command for.
   * @param callback  Runnable to be added to the shared command.
   */
  public static void addCommand(String cmdName, Subsystem subsystem, Runnable callback)
  {
    var key               = subsystem.getName() + "/" + cmdName;
    var existingCallbacks = commandCallbacks.getOrDefault(key, new ArrayList<>());
    existingCallbacks.add(callback);
    commandCallbacks.put(key, existingCallbacks);
    // Create Command and publish it to NT
    if (!commandExists(cmdName, subsystem))
    {addCommandToNT(cmdName, subsystem);}
  }

  /**
   * Check if a command exists.
   *
   * @param cmdName   Command name.
   * @param subsystem Subsystem.
   * @return True if command exists.
   */
  public static boolean commandExists(String cmdName, Subsystem subsystem)
  {
    var key = subsystem.getName() + "/" + cmdName;
    return commands.containsKey(key);
  }

}
