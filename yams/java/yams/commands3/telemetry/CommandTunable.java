// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.telemetry;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.tunable.ComplexTunable;
import org.wpilib.tunable.TunableTable;

/**
 * Adapts an {@link Command} for publishing to NetworkTables via
 * {@link org.wpilib.tunable.Tunables}. Unlike {@code org.wpilib.command2.Command}, {@link Command} does
 * not implement {@link ComplexTunable} itself, so this wrapper exposes a "running" boolean that
 * schedules the command when set true and cancels it when set false.
 */
public class CommandTunable implements ComplexTunable {
  /** Command being published. */
  private final Command command;

  /**
   * Wrap a {@link Command} for publishing to NetworkTables.
   *
   * @param command {@link Command} to wrap.
   */
  public CommandTunable(Command command) {
    this.command = command;
  }

  @Override
  public String getTunableType() {
    return "Command";
  }

  @Override
  public void publishTunable(TunableTable table) {
    table.publishBoolean("running", () -> Scheduler.getDefault().isRunning(command), value -> {
      if (value) {
        Scheduler.getDefault().schedule(command);
      } else {
        Scheduler.getDefault().cancel(command);
      }
    });
  }
}
