// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * A command from a PathPlanner {@code .auto} file or event marker, before it is turned into a robot
 * command. Groups hold other commands; the leaves follow a path, run a named command, or wait.
 */
public sealed interface PathPlannerCommand
{

  /** Run the commands one after another. */
  record Sequential(List<PathPlannerCommand> commands) implements PathPlannerCommand
  {

  }

  /** Run the commands together until all of them finish. */
  record Parallel(List<PathPlannerCommand> commands) implements PathPlannerCommand
  {

  }

  /** Run the commands together until any of them finishes. */
  record Race(List<PathPlannerCommand> commands) implements PathPlannerCommand
  {

  }

  /** Run the commands together until the first one finishes. */
  record Deadline(List<PathPlannerCommand> commands) implements PathPlannerCommand
  {

  }

  /** Follow the path with this name. */
  record FollowPath(String pathName) implements PathPlannerCommand
  {

  }

  /** Run the command registered with {@code NamedCommands} under this name. */
  record Named(String name) implements PathPlannerCommand
  {

  }

  /** Wait for a number of seconds. */
  record Wait(double seconds) implements PathPlannerCommand
  {

  }

  /**
   * Parse a command from its JSON form, {@code {"type": ..., "data": {...}}}.
   *
   * @param json Parsed JSON object.
   * @return The command.
   * @throws IllegalArgumentException if the command type is unknown.
   */
  @SuppressWarnings("unchecked")
  static PathPlannerCommand fromJson(Map<String, Object> json)
  {
    String              type = (String) json.get("type");
    Map<String, Object> data = (Map<String, Object>) json.get("data");
    return switch (type)
    {
      case "sequential" -> new Sequential(children(data));
      case "parallel" -> new Parallel(children(data));
      case "race" -> new Race(children(data));
      case "deadline" -> new Deadline(children(data));
      case "path" -> new FollowPath((String) data.get("pathName"));
      // The GUI leaves the name null on a named command that has not been picked yet.
      case "named" -> new Named(Optional.ofNullable((String) data.get("name")).orElse(""));
      case "wait" -> new Wait(((Number) data.get("waitTime")).doubleValue());
      default -> throw new IllegalArgumentException("Unknown PathPlanner command type: " + type);
    };
  }

  @SuppressWarnings("unchecked")
  private static List<PathPlannerCommand> children(Map<String, Object> data)
  {
    return ((List<Map<String, Object>>) data.get("commands")).stream()
                                                              .map(PathPlannerCommand::fromJson)
                                                              .toList();
  }

  /**
   * The first path this command follows, if any. An auto that resets odometry starts at the
   * starting pose of this path.
   *
   * @return Name of the first path followed.
   */
  default Optional<String> firstPathName()
  {
    return switch (this)
    {
      case FollowPath path -> Optional.of(path.pathName());
      case Sequential group -> firstPathName(group.commands());
      case Parallel group -> firstPathName(group.commands());
      case Race group -> firstPathName(group.commands());
      case Deadline group -> firstPathName(group.commands());
      case Named named -> Optional.empty();
      case Wait wait -> Optional.empty();
    };
  }

  private static Optional<String> firstPathName(List<PathPlannerCommand> commands)
  {
    return commands.stream().map(PathPlannerCommand::firstPathName).flatMap(Optional::stream).findFirst();
  }
}
