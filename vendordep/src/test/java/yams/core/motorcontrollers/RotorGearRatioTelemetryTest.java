// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Meters;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Distance;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.gearing.Sprocket;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.helpers.MockHardwareExtension;

/**
 * Every motor controller logs its gear ratio under {@code rotor/gearRatio}, and a linear mechanism
 * its circumference under {@code measurement/circumference}, at every telemetry level, so a log can
 * turn a rotor reading into the mechanism's: divide by the ratio, and times the circumference for
 * meters.
 */
public class RotorGearRatioTelemetryTest {
  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static Stream<Controller> controllers() {
    return ClosedLoopCases.controllers();
  }

  /** Publish one motor controller's telemetry, and read its gear ratio back. */
  private static double publishedRatio(Controller controller, String name, MechanismGearing gearing) {
    return published(controller, name, gearing, null, "rotor/gearRatio");
  }

  /** Publish one motor controller's telemetry, and read a field back: NaN when it is not published. */
  private static double published(Controller controller, String name, MechanismGearing gearing,
      Distance circumference, String field) {
    final yams.commands2.config.SmartMotorControllerConfig cfg =
        ClosedLoopCases.baseConfig("RotorGearRatio " + name).withClosedLoopController(1, 0, 0);
    if (circumference != null) {
      cfg.withMechanismCircumference(circumference);
    }
    final SmartMotorController smc = ClosedLoopCases.create(controller, cfg);
    try {
      if (gearing != null) {
        smc.getConfig().withGearing(gearing);
      }
      final NetworkTable telemetry = NetworkTableInstance.getDefault().getTable("RotorGearRatioTest");
      final NetworkTable tuning = NetworkTableInstance.getDefault().getTable("RotorGearRatioTestTuning");
      smc.setupTelemetry(telemetry, tuning);
      smc.updateTelemetry();
      return telemetry.getSubTable("RotorGearRatio " + name).getEntry(field).getDouble(Double.NaN);
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  /** The motor controller's own reduction, published at LOW, the lowest level. */
  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void logsTheReduction(Controller controller) {
    assertEquals(controller.ratio(), publishedRatio(controller, controller.toString(), null), 1e-9,
        controller + " should log its " + controller.ratio() + ":1 reduction");
  }

  /** Stages and sprockets combine as YAMS uses them, which is easy to get wrong by hand. */
  @Test
  void logsTheRatioYamsUses() {
    final Controller spark = ClosedLoopCases.controllers().filter(c -> c.name().equals("SparkMax")).findFirst()
        .orElseThrow();
    // 3:1 and 4:1 stages, then a 12 tooth sprocket driving a 24 tooth one: 3 x 4 x 2 = 24.
    assertEquals(24, publishedRatio(spark, "stages and sprocket",
        new MechanismGearing(GearBox.fromReductionStages(3, 4), Sprocket.fromStages("12:24"))), 1e-9);
  }

  /** A linear mechanism logs its circumference; a rotating one does not publish the field. */
  @Test
  void logsTheCircumferenceOfALinearMechanism() {
    final Controller spark = ClosedLoopCases.controllers().filter(c -> c.name().equals("SparkMax")).findFirst()
        .orElseThrow();
    assertEquals(0.15, published(spark, "linear", null, Meters.of(0.15), "measurement/circumference"), 1e-9);
    assertTrue(Double.isNaN(published(spark, "rotating", null, null, "measurement/circumference")),
        "a mechanism without a circumference should not publish one");
  }
}
