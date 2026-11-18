// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.datalog.DataLogReader;
import org.wpilib.datalog.DataLogRecord;
import org.wpilib.datalog.DoubleLogEntry;
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.DataLogManager;
import yams.helpers.MockHardwareExtension;

/**
 * YAMS' dedicated DataLog entries must be stamped on the DataLog's own clock, the one the
 * NetworkTables mirror and every other entry use, so a recorded run plays back over time instead of
 * every value landing at one instant.
 */
public class DataLogTimestampTest {
  @TempDir
  Path logDir;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    SimHooks.pauseTiming();
  }

  @AfterEach
  void endTest() {
    DataLogManager.stop();
    SimHooks.resumeTiming();
    MockHardwareExtension.afterAll();
  }

  @Test
  void valuesAreStampedOnTheDataLogClock() throws IOException {
    DataLogManager.start(logDir.toString(), "timestamps.wpilog");
    DoubleTelemetry<String> telemetry = new DoubleTelemetry<>("value", 0, "value", false, "units");
    telemetry.enable();
    telemetry.setupDataLog("Test");
    // A plain entry, stamped by the DataLog itself, as the NetworkTables mirror is.
    DoubleLogEntry reference = new DoubleLogEntry(DataLogManager.getLog(), "Reference");

    for (int i = 0; i < 4; i++) {
      telemetry.set(i);
      reference.append(i);
      SimHooks.stepTiming(0.5);
    }
    DataLogManager.getLog().flush();

    // The DataLog writes on a background thread and can hold back its newest records, so wait for
    // the first three of each.
    Path file = logDir.resolve("timestamps.wpilog");
    List<Long> yams = List.of();
    List<Long> ref = List.of();
    for (int tries = 0; tries < 50 && (yams.size() < 3 || ref.size() < 3); tries++) {
      sleep(100);
      yams = timestampsOf(file, "Test/units/value");
      ref = timestampsOf(file, "Reference");
    }
    assertTrue(yams.size() >= 3 && ref.size() >= 3, "records reached the file: " + yams + " " + ref);
    long halfSecond = ref.get(1) - ref.get(0);
    assertTrue(halfSecond > 0, "the reference entry advances: " + ref);
    for (int i = 0; i < 3; i++) {
      assertTrue(Math.abs(yams.get(i) - ref.get(i)) < halfSecond / 100,
                 "record " + i + " is on the DataLog clock: YAMS " + yams + ", reference " + ref);
    }
  }

  private static void sleep(long ms) {
    try {
      Thread.sleep(ms);
    } catch (InterruptedException e) {
      Thread.currentThread().interrupt();
    }
  }

  /** Timestamps of every data record for one entry, in file order. */
  private static List<Long> timestampsOf(Path file, String name) throws IOException {
    if (!Files.exists(file)) {
      return List.of();
    }
    DataLogReader reader = new DataLogReader(file.toString());
    int entry = -1;
    List<Long> stamps = new ArrayList<>();
    for (DataLogRecord record : reader) {
      if (record.isStart() && record.getStartData().name.equals(name)) {
        entry = record.getStartData().entry;
      } else if (!record.isControl() && record.getEntry() == entry) {
        stamps.add(record.getTimestamp());
      }
    }
    return stamps;
  }
}
