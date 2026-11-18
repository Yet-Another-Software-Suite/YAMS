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
import org.wpilib.simulation.SimHooks;
import org.wpilib.system.DataLogManager;
import org.wpilib.system.Timer;
import yams.helpers.MockHardwareExtension;

/**
 * YAMS' dedicated DataLog entries must be stamped in microseconds at the time each value is
 * written, so a recorded run plays back over time instead of every value landing at one instant.
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
  void valuesAreStampedWhenTheyAreWritten() throws IOException {
    DataLogManager.start(logDir.toString(), "timestamps.wpilog");
    DoubleTelemetry<String> telemetry = new DoubleTelemetry<>("value", 0, "value", false, "units");
    telemetry.enable();
    telemetry.setupDataLog("Test");

    long expectedFirst = (long) (Timer.getTimestamp() * 1e6);
    telemetry.set(1);
    SimHooks.stepTiming(0.5);
    telemetry.set(2);
    SimHooks.stepTiming(0.5);
    telemetry.set(3);
    // The DataLog's writer can hold back its newest record, so write one more and check the first three.
    SimHooks.stepTiming(0.5);
    telemetry.set(4);
    DataLogManager.getLog().flush();

    // The DataLog writes on a background thread, so wait for the records to reach the file.
    List<Long> stamps = List.of();
    for (int tries = 0; tries < 50 && stamps.size() < 3; tries++) {
      sleep(100);
      stamps = timestampsOf(logDir.resolve("timestamps.wpilog"), "Test/units/value");
    }
    assertTrue(stamps.size() >= 3, "one record per value: " + stamps);
    assertTrue(Math.abs(stamps.get(0) - expectedFirst) < 1_000,
               "stamped in microseconds of Timer.getTimestamp(): " + stamps.get(0) + " vs " + expectedFirst);
    // Half a second of simulated time between writes is 500,000 microseconds.
    for (int i = 1; i < 3; i++) {
      long step = stamps.get(i) - stamps.get(i - 1);
      assertTrue(Math.abs(step - 500_000) < 50_000,
                 "expected about 500000 us between records, got " + step + " us (" + stamps + ")");
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
