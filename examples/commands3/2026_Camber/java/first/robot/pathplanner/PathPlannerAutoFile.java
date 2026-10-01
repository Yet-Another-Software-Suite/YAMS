// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Map;
import org.wpilib.system.Filesystem;

/**
 * An auto built in the PathPlanner GUI, read from {@code deploy/pathplanner/autos/<name>.auto}.
 *
 * @param name      Name of the auto.
 * @param command   The auto's command tree.
 * @param resetOdom Whether the auto resets odometry to the starting pose of its first path.
 */
public record PathPlannerAutoFile(String name, PathPlannerCommand command, boolean resetOdom)
{

  /**
   * Load an auto from the deploy directory.
   *
   * @param autoName Name of the auto in the PathPlanner GUI, without the {@code .auto} extension.
   * @return The auto.
   * @throws IOException if the file cannot be read.
   */
  @SuppressWarnings("unchecked")
  public static PathPlannerAutoFile fromAutoFile(String autoName) throws IOException
  {
    Path file = Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos")
                          .resolve(autoName + ".auto");
    Map<String, Object> json      = PathPlannerJson.readObject(Files.readString(file));
    Object              resetOdom = json.get("resetOdom");
    return new PathPlannerAutoFile(autoName,
                                   PathPlannerCommand.fromJson((Map<String, Object>) json.get("command")),
                                   resetOdom == null || (Boolean) resetOdom);
  }
}
