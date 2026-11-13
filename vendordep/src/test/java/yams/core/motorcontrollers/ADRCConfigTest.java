// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.ADRCController;
import yams.core.math.TrackingDifferentiator;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.helpers.MockHardwareExtension;

/**
 * {@link ADRCConfig}: the values it keeps, the defaults it builds with, the {@code b0} it derives,
 * and how {@code withADRCController} replaces the other closed loop controllers and feedforwards of
 * its slot.
 */
class ADRCConfigTest {
  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
  }

  private static yams.commands2.config.SmartMotorControllerConfig smcConfig() {
    return new yams.commands2.config.SmartMotorControllerConfig().withGearing(
        new MechanismGearing(GearBox.fromReductionStages(12)));
  }

  @Test
  void unsetValuesAreEmptyAndBuildWithTheirDefaults() {
    final ADRCConfig cfg = new ADRCConfig();
    assertTrue(cfg.getSystemOrder().isEmpty());
    assertTrue(cfg.getSettlingTime().isEmpty());
    assertTrue(cfg.getBandwidthScale().isEmpty());
    assertTrue(cfg.getB0().isEmpty());
    assertTrue(cfg.getInputDelay().isEmpty());
    assertTrue(cfg.getTdMethod().isEmpty());
    assertTrue(cfg.getTdParams().isEmpty());
    assertFalse(cfg.isCascaded());
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.buildController(DCMotor.getNEO(1),
            smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20)),
        "there is no default system order");

    final ADRCController controller = cfg.withSystemOrder(SystemOrder.FIRST).withB0(10)
        .buildController(DCMotor.getNEO(1), smcConfig(), Milliseconds.of(20));
    assertEquals(ADRCConfig.DEFAULT_SETTLING_TIME, controller.getSettlingTime());
    assertEquals(ADRCConfig.DEFAULT_BANDWIDTH_SCALE, controller.getBandwidthScale());
  }

  @Test
  void keepsEveryValueItIsGiven() {
    final ADRCConfig cfg = new ADRCConfig()
        .withSystemOrder(SystemOrder.SECOND)
        .withSettlingTime(Seconds.of(0.4))
        .withBandwidthScale(1.5)
        .withB0(42)
        .withInputDelay(Seconds.of(0.04))
        .withCascaded(true)
        .withTrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.5);
    assertEquals(SystemOrder.SECOND, cfg.getSystemOrder().orElseThrow());
    assertEquals(0.4, cfg.getSettlingTime().orElseThrow().in(Seconds));
    assertEquals(1.5, cfg.getBandwidthScale().orElseThrow());
    assertEquals(42, cfg.getB0().orElseThrow());
    assertEquals(0.04, cfg.getInputDelay().orElseThrow().in(Seconds));
    assertTrue(cfg.isCascaded());
    assertEquals(TrackingDifferentiator.Method.LD, cfg.getTdMethod().orElseThrow());
    assertArrayEquals(new double[] {0.5}, cfg.getTdParams().orElseThrow());

    cfg.withoutTrackingDifferentiator();
    assertTrue(cfg.getTdMethod().isEmpty());
    assertTrue(cfg.getTdParams().isEmpty());
  }

  @Test
  void refusesValuesTheControllerCannotUse() {
    final ADRCConfig cfg = new ADRCConfig()
        .withSystemOrder(SystemOrder.FIRST)
        .withSettlingTime(Seconds.of(0.3));
    assertThrows(IllegalArgumentException.class, () -> cfg.withSettlingTime(Seconds.of(-1)));
    assertThrows(IllegalArgumentException.class, () -> cfg.withSettlingTime(Seconds.of(0)));
    assertThrows(IllegalArgumentException.class, () -> cfg.withBandwidthScale(0));
    assertThrows(IllegalArgumentException.class, () -> cfg.withB0(0));
    assertThrows(IllegalArgumentException.class, () -> cfg.withB0(Double.NaN));
    assertThrows(IllegalArgumentException.class, () -> cfg.withInputDelay(Seconds.of(-1)));
    // A refused value leaves the config as it was.
    assertEquals(0.3, cfg.getSettlingTime().orElseThrow().in(Seconds));
    assertTrue(cfg.getB0().isEmpty());
  }

  @Test
  void derivesB0FromTheMotorGearingAndInertia() {
    final DCMotor motor = DCMotor.getNEO(1);
    final ADRCConfig cfg = new ADRCConfig().withSystemOrder(SystemOrder.SECOND);
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.resolveB0(motor, smcConfig()), "the placeholder inertia is never used");
    final double expected = 12 * motor.Kt / (2 * Math.PI * motor.R * 0.1);
    assertEquals(expected,
        cfg.resolveB0(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1))), 1e-9);
    assertEquals(7.0, cfg.withB0(7.0).resolveB0(motor, smcConfig()), "a b0 that is set wins");
  }

  @Test
  void b0FromTheMotorAndGearingIsTheMechanismsSpeedPerVolt() {
    final DCMotor motor = DCMotor.getNEO(1);
    final ADRCConfig cfg = new ADRCConfig().withB0(motor, new MechanismGearing(GearBox.fromReductionStages(3, 4)));
    assertEquals(motor.Kv / (2 * Math.PI * 12), cfg.getB0().orElseThrow(), 1e-12,
        "the motor's free speed per volt, in mechanism rotations per second, through a 12:1 reduction");
  }

  @Test
  void derivesB0PerMeterForALinearClosedLoop() {
    final DCMotor motor = DCMotor.getNEO(1);
    final double circumference = 0.1;
    final yams.commands2.config.SmartMotorControllerConfig smc =
        smcConfig()
            .withMechanismCircumference(Meters.of(circumference))
            .withMomentOfInertia(KilogramSquareMeters.of(0.01))
            .withLinearClosedLoopController(true);
    final double perRotation = 12 * motor.Kt / (2 * Math.PI * motor.R * 0.01);
    assertEquals(
        perRotation * circumference, new ADRCConfig().resolveB0(motor, smc), 1e-9);
  }

  @Test
  void buildsAControllerFromTheConfig() {
    final ADRCController controller =
        new ADRCConfig()
            .withSystemOrder(SystemOrder.SECOND)
            .withSettlingTime(Seconds.of(0.5))
            .withB0(10)
            .buildController(DCMotor.getNEO(1),
                smcConfig().withClosedLoopControllerMaximumVoltage(Volts.of(9)),
                Milliseconds.of(10));
    assertEquals(2, controller.getSystemOrder());
    assertEquals(0.5, controller.getSettlingTime().in(Seconds));
    assertEquals(10, controller.getB0());
    assertEquals(9, controller.getMaxOutput().in(Volts), "the motor config's closed loop voltage limit");
    assertEquals(-9, controller.getMinOutput().in(Volts));
    assertEquals(0, controller.getSlewRateLimit().in(Volts.per(Second)), "the motor controller ramps its output");
    final ADRCController fallback =
        new ADRCConfig()
            .withSystemOrder(SystemOrder.FIRST)
            .withB0(10)
            .buildController(DCMotor.getNEO(1), smcConfig(), Milliseconds.of(20));
    assertEquals(
        DCMotor.getNEO(1).nominalVoltage, fallback.getMaxOutput().in(Volts), "the motor's nominal voltage");
  }

  @Test
  void replacesThePidLqrAndFeedforwardsOfItsSlot() {
    final ClosedLoopControllerSlot slot = ClosedLoopControllerSlot.SLOT_0;
    final yams.commands2.config.SmartMotorControllerConfig smc =
        smcConfig()
            .withClosedLoopController(1, 0, 0)
            .withFeedforward(new SimpleMotorFeedforward(0, 1, 0))
            .withFeedforward(new ArmFeedforward(0, 1, 0, 0))
            .withFeedforward(new ElevatorFeedforward(0, 1, 0, 0))
            .withADRCController(new ADRCConfig().withSystemOrder(SystemOrder.FIRST));
    assertTrue(smc.getADRCController(slot).isPresent());
    assertTrue(smc.getPID(slot).isEmpty());
    assertTrue(smc.getLQRClosedLoopController().isEmpty());
    assertTrue(smc.getSimpleFeedforward(slot).isEmpty());
    assertTrue(smc.getArmFeedforward(slot).isEmpty());
    assertTrue(smc.getElevatorFeedforward(slot).isEmpty());

    smc.withClosedLoopController(1, 0, 0);
    assertTrue(smc.getADRCController(slot).isEmpty(), "a PID replaces the ADRC");
    assertTrue(smc.getPID(slot).isPresent());
  }

  @Test
  void onlyItsOwnSlotIsReplaced() {
    final yams.commands2.config.SmartMotorControllerConfig smc =
        smcConfig()
            .withClosedLoopController(1, 0, 0, ClosedLoopControllerSlot.SLOT_1)
            .withADRCController(new ADRCConfig().withSystemOrder(SystemOrder.FIRST),
                ClosedLoopControllerSlot.SLOT_0);
    assertTrue(smc.getADRCController(ClosedLoopControllerSlot.SLOT_0).isPresent());
    assertTrue(smc.getPID(ClosedLoopControllerSlot.SLOT_1).isPresent());
    assertTrue(smc.getADRCController(ClosedLoopControllerSlot.SLOT_1).isEmpty());
  }

  @Test
  void aCopyOfTheMotorConfigGetsItsOwnADRCConfig() {
    final yams.commands2.config.SmartMotorControllerConfig smc = smcConfig().withADRCController(
        new ADRCConfig().withSystemOrder(SystemOrder.FIRST).withB0(3));
    final yams.commands2.config.SmartMotorControllerConfig copy =
        (yams.commands2.config.SmartMotorControllerConfig) smc.clone();
    assertNotSame(smc.getADRCController().orElseThrow(), copy.getADRCController().orElseThrow());
    copy.getADRCController().orElseThrow().withB0(9);
    assertEquals(3, smc.getADRCController().orElseThrow().getB0().orElseThrow(),
        "changing the copy leaves the original");
  }
}
