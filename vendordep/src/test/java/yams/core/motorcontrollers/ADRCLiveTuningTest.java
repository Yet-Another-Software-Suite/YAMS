// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Seconds;

import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.ADRCController;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.MockHardwareExtension;

/**
 * Live tuning of an ADRC, on every motor controller. The tuning table shows the ADRC's values, and
 * values entered there are kept in the motor controller's {@link ADRCConfig}, like a tuned PID, and
 * the ADRC is reset to them, keeping its state estimate; applying the motor config again keeps them
 * too. A value the ADRC cannot use leaves it as it was.
 */
public class ADRCLiveTuningTest {
  private static final double kTolerance = 1e-9;

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

  private static void assertShown(NetworkTable tuning, String key, double expected, String name) {
    assertEquals(expected, tuning.getEntry(key).getDouble(Double.NaN), kTolerance, name + ": " + key + " shown");
  }

  private static void enter(NetworkTable tuning, String key, double value) {
    tuning.getEntry(key).setDouble(value);
  }

  /** The ADRC's tuning, as the controller runs it. */
  private static void assertRunning(
      ADRCController adrc, double b0, double settlingTime, double bandwidthScale, String name) {
    assertEquals(b0, adrc.getB0(), kTolerance, name + ": running b0");
    assertEquals(settlingTime, adrc.getSettlingTime().in(Seconds), kTolerance, name + ": running settling time");
    assertEquals(bandwidthScale, adrc.getBandwidthScale(), kTolerance, name + ": running bandwidth scale");
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("controllers")
  void tuningIsKeptInTheConfigAndFollowedByTheADRC(Controller controller) {
    final String name = "ADRCLiveTuningTest " + controller;
    final SmartMotorController smc = ClosedLoopCases.create(
        controller,
        ClosedLoopCases.baseConfig(name)
            .withTelemetry(name, TelemetryVerbosity.HIGH)
            .withMomentOfInertia(KilogramSquareMeters.of(0.1))
            .withStartingPosition(Degrees.of(0))
            .withADRCController(new ADRCConfig()
                .withSystemOrder(SystemOrder.SECOND)
                .withSettlingTime(Seconds.of(0.6))));
    try {
      final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("ADRCLiveTuningTestTuning");
      smc.setupTelemetry(NetworkTableInstance.getDefault().getTable("ADRCLiveTuningTest"), tuningTable);
      final NetworkTable tuning = tuningTable.getSubTable(name);
      final ADRCController adrc = smc.getADRCController().orElseThrow();
      final double derivedB0 = adrc.getB0();

      // The tuning table shows what the ADRC runs with.
      assertShown(tuning, "closedloop/adrc/b0", derivedB0, name);
      assertShown(tuning, "closedloop/adrc/settlingTime", 0.6, name);
      assertShown(tuning, "closedloop/adrc/bandwidthScale", 1, name);
      // The output limit and ramp are the motor config's, tuned with every other closed loop's.
      assertFalse(tuning.containsKey("closedloop/adrc/maxVoltage"), name + ": no ADRC maximum voltage");
      assertFalse(tuning.containsKey("closedloop/adrc/slewRateLimit"), name + ": no ADRC slew rate limit");

      // Enter new values and apply them, holding the mechanism where it is.
      enter(tuning, "closedloop/adrc/b0", 0.8 * derivedB0);
      enter(tuning, "closedloop/adrc/settlingTime", 0.4);
      enter(tuning, "closedloop/adrc/bandwidthScale", 1.5);
      enter(tuning, "closedloop/setpoint/position", 0);
      final double disturbance = adrc.getEstimatedDisturbance();
      smc.applyTuningValues();

      // The config holds exactly the entered values.
      final ADRCConfig adrcConfig = smc.getConfig().getADRCController().orElseThrow();
      assertEquals(0.8 * derivedB0, adrcConfig.getB0().orElseThrow(), kTolerance, name + ": config b0");
      assertEquals(0.4, adrcConfig.getSettlingTime().orElseThrow().in(Seconds), kTolerance, name + ": config settling time");
      assertEquals(1.5, adrcConfig.getBandwidthScale().orElseThrow(), kTolerance, name + ": config bandwidth scale");

      // The ADRC runs them, keeping the disturbance it estimated.
      assertRunning(adrc, 0.8 * derivedB0, 0.4, 1.5, name);
      assertEquals(disturbance, adrc.getEstimatedDisturbance(), 1e-6, name + ": disturbance kept");

      // Applying the motor config again builds the ADRC from the tuned config.
      smc.applyConfig(smc.getConfig());
      assertRunning(smc.getADRCController().orElseThrow(), 0.8 * derivedB0, 0.4, 1.5, name + " reapplied");

      // A value the ADRC can't use leaves it as it was.
      final ADRCController rebuilt = smc.getADRCController().orElseThrow();
      enter(tuning, "closedloop/adrc/settlingTime", -1);
      smc.applyTuningValues();
      assertRunning(rebuilt, 0.8 * derivedB0, 0.4, 1.5, name + " rejected");
    } finally {
      ClosedLoopCases.close(smc);
    }
  }
}
