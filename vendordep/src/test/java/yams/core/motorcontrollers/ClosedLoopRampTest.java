// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Optional;
import java.util.function.BiFunction;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.units.measure.Time;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.LQRConfig;
import yams.core.math.LQRController;
import yams.core.motorcontrollers.ClosedLoopCases.Controller;
import yams.helpers.MockHardwareExtension;

/**
 * {@code withClosedLoopRampRate} ramps the output of a closed loop that runs on the SystemCore: an
 * ADRC (written by FRC Team 8331) or an LQR, on every motor controller. Such a loop sends the motor
 * controller a voltage, which the motor controller treats as open loop, so its own closed loop ramp
 * never applies; YAMS ramps the output instead.
 */
public class ClosedLoopRampTest {
  /** Time from 0 V to 12 V. */
  private static final double kRampSeconds = 0.5;
  /** How fast the ramp lets the output change, volts per second. */
  private static final double kRampVoltsPerSecond = 12.0 / kRampSeconds;
  /** Slack for the applied voltage, which a motor controller reports in steps. */
  private static final double kSlackVolts = 0.75;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12.0);
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** A SystemCore closed loop controller, added to a motor config. */
  private enum Loop {
    ADRC((controller, cfg) -> cfg.withMomentOfInertia(KilogramSquareMeters.of(0.1))
        .withADRCController(new ADRCConfig().withSystemOrder(SystemOrder.SECOND).withSettlingTime(Seconds.of(0.6)))),
    LQR((controller, cfg) -> cfg.withClosedLoopController(new LQRController(
        new LQRConfig(controller.motor(), controller.gearing(), KilogramSquareMeters.of(0.02))
            .withArm(Radians.of(0.01), RadiansPerSecond.of(0.5), Radians.of(0.05), RadiansPerSecond.of(0.5),
                Radians.of(0.01))
            .withControlEffort(Volts.of(12))
            .withMaxVoltage(Volts.of(12)))));

    private final BiFunction<Controller, SmartMotorControllerConfig, SmartMotorControllerConfig> add;

    Loop(BiFunction<Controller, SmartMotorControllerConfig, SmartMotorControllerConfig> add) {
      this.add = add;
    }
  }

  private static Stream<Arguments> controllersAndLoops() {
    return ClosedLoopCases.controllers()
        .flatMap(controller -> Stream.of(Loop.values()).map(loop -> Arguments.of(controller, loop)));
  }

  /**
   * Step a whole rotation with the given ramp rate, and return how far above the ramp, from 0 V at
   * 24 V/s, the applied voltage got.
   */
  private static double voltsOverTheRamp(Controller controller, Loop loop, Optional<Time> rampRate) {
    final String name = "ClosedLoopRampTest " + controller + " " + loop + (rampRate.isPresent() ? "" : " unramped");
    final SmartMotorControllerConfig cfg = ClosedLoopCases.baseConfig(name).withStartingPosition(Degrees.of(0));
    rampRate.ifPresent(cfg::withClosedLoopRampRate);
    final SmartMotorController smc = ClosedLoopCases.create(controller, loop.add.apply(controller, cfg));
    try {
      assertTrue(smc.usesSystemCoreController(), name + " should run its closed loop on the SystemCore");
      // A whole rotation away, so the loop asks for a large output at once.
      final double[] worstOverRamp = {0};
      ClosedLoopCases.run(controller, smc, 0.5, () -> smc.setPosition(Rotations.of(1)), time -> {
        final double volts = Math.abs(smc.getVoltage().in(Volts));
        worstOverRamp[0] = Math.max(worstOverRamp[0], volts - kRampVoltsPerSecond * time);
      });
      System.out.println(name + ": at most " + worstOverRamp[0] + " V over the ramp");
      return worstOverRamp[0];
    } finally {
      ClosedLoopCases.close(smc);
    }
  }

  @ParameterizedTest(name = "{0} {1}")
  @MethodSource("controllersAndLoops")
  void theClosedLoopRampLimitsHowFastTheOutputChanges(Controller controller, Loop loop) {
    final double ramped = voltsOverTheRamp(controller, loop, Optional.of(Seconds.of(kRampSeconds)));
    assertTrue(ramped <= kSlackVolts,
        controller + " " + loop + ": the output rose " + ramped + " V faster than " + kRampVoltsPerSecond + " V/s");
    // Without the ramp the same step rises faster, so the ramp is what held it back. A Talon is
    // simulated in real time, so its first outputs come late, and only the ramped run is checked.
    if (!controller.realTime()) {
      final double unramped = voltsOverTheRamp(controller, loop, Optional.empty());
      assertTrue(unramped > kSlackVolts,
          controller + " " + loop + ": without a ramp the output should rise faster than the ramp, but was only "
              + unramped + " V over it");
    }
  }
}
