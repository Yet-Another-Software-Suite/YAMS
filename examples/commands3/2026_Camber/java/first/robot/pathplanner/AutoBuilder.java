// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import static org.wpilib.units.Units.Seconds;

import first.robot.utils.AllianceFlipUtil;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.Function;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;

/**
 * Builds commands from PathPlanner autos and paths, like PathPlannerLib's {@code AutoBuilder}.
 * PathPlannerLib is built on commands v2, so autos become coroutine commands here instead. Configure
 * it once with the drivetrain before building any autos.
 */
public final class AutoBuilder
{

  private static Function<PathPlannerPath, Command> followPath;
  private static Consumer<Pose2d>                   resetPose;
  private static BooleanSupplier                    shouldFlipPath;

  private AutoBuilder()
  {
  }

  /**
   * Configure the drivetrain that paths drive.
   *
   * @param followPath     Builds a command that follows a path.
   * @param resetPose      Resets odometry to a pose, blue alliance origin.
   * @param shouldFlipPath Whether paths should be mirrored for the red alliance.
   */
  public static void configure(Function<PathPlannerPath, Command> followPath, Consumer<Pose2d> resetPose,
                               BooleanSupplier shouldFlipPath)
  {
    AutoBuilder.followPath = followPath;
    AutoBuilder.resetPose = resetPose;
    AutoBuilder.shouldFlipPath = shouldFlipPath;
  }

  /** @return Whether paths should be mirrored for the red alliance right now. */
  public static boolean shouldFlip()
  {
    return shouldFlipPath.getAsBoolean();
  }

  /**
   * Build the command for an auto. Named commands must be registered first. The files are read
   * now, so build autos while the robot is disabled.
   *
   * @param autoName Name of the auto in the PathPlanner GUI.
   * @return Command that runs the auto, or does nothing if the auto cannot be loaded.
   */
  public static Command buildAuto(String autoName)
  {
    try
    {
      PathPlannerAutoFile auto    = PathPlannerAutoFile.fromAutoFile(autoName);
      Command             command = buildCommand(auto.command());
      Pose2d startingPose = auto.resetOdom() && auto.command().firstPathName().isPresent()
                            ? PathPlannerPath.fromPathFile(auto.command().firstPathName().get()).getStartingPose()
                            : null;
      return Command.noRequirements(coroutine -> {
        if (startingPose != null)
        {
          resetPose.accept(shouldFlip() ? AllianceFlipUtil.flip(startingPose) : startingPose);
        }
        coroutine.await(command);
      }).named(autoName);
    } catch (IOException | RuntimeException e)
    {
      DriverStationErrors.reportError("Could not load PathPlanner auto " + autoName + ": " + e, e.getStackTrace());
      return Command.noRequirements(coroutine -> {}).named(autoName + " (failed to load)");
    }
  }

  /**
   * Build a command that follows a path, running its event markers along the way.
   *
   * @param pathName Name of the path in the PathPlanner GUI.
   * @return Command that follows the path, or does nothing if the path cannot be loaded.
   */
  public static Command followPath(String pathName)
  {
    try
    {
      return followPath.apply(PathPlannerPath.fromPathFile(pathName));
    } catch (IOException | RuntimeException e)
    {
      DriverStationErrors.reportError("Could not load PathPlanner path " + pathName + ": " + e, e.getStackTrace());
      return Command.noRequirements(coroutine -> {}).named("Follow " + pathName + " (failed to load)");
    }
  }

  /**
   * Build the robot command for a PathPlanner command from an auto or event marker.
   *
   * @param command PathPlanner command.
   * @return Robot command.
   */
  public static Command buildCommand(PathPlannerCommand command)
  {
    return switch (command)
    {
      case PathPlannerCommand.Sequential group ->
      {
        List<Command> children = children(group.commands());
        yield Command.noRequirements(coroutine -> {
          for (Command child : children)
          {
            coroutine.await(child);
          }
        }).named("Sequential");
      }
      case PathPlannerCommand.Parallel group ->
      {
        List<Command> children = children(group.commands());
        yield Command.noRequirements(coroutine -> coroutine.awaitAll(children)).named("Parallel");
      }
      case PathPlannerCommand.Race group ->
      {
        List<Command> children = children(group.commands());
        yield Command.noRequirements(coroutine -> coroutine.awaitAny(children)).named("Race");
      }
      case PathPlannerCommand.Deadline group ->
      {
        List<Command> children = children(group.commands());
        yield Command.noRequirements(coroutine -> {
          if (children.isEmpty())
          {
            return;
          }
          // The others are canceled when the deadline finishes and this command ends.
          coroutine.fork(children.subList(1, children.size()));
          coroutine.await(children.get(0));
        }).named("Deadline");
      }
      case PathPlannerCommand.FollowPath path -> followPath(path.pathName());
      case PathPlannerCommand.Named named -> NamedCommands.getCommand(named.name());
      case PathPlannerCommand.Wait wait -> Command.waitFor(Seconds.of(wait.seconds())).named("Wait " + wait.seconds());
    };
  }

  private static List<Command> children(List<PathPlannerCommand> commands)
  {
    return Arrays.asList(commands.stream().map(AutoBuilder::buildCommand).toArray(Command[]::new));
  }
}
