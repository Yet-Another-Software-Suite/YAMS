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
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
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
  void theBackEmfIsModeledWhereB0IsAnAccelerationPerVolt() {
    final DCMotor motor = DCMotor.getNEO(1);
    final double tau12 = 0.1 * motor.R / (12 * 12 * motor.Kt / motor.Kv);
    final ADRCConfig pivot = new ADRCConfig().withSystemOrder(SystemOrder.PIVOT);
    pivot.buildController(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20));
    assertTrue(pivot.isBackEmfModeled());
    assertEquals(tau12, pivot.getModeledBackEmfTimeConstant().orElseThrow().in(Seconds), 1e-12,
        "a second order position loop models its back-EMF time constant");
    pivot.withBackEmfModel(false);
    assertTrue(pivot.getModeledBackEmfTimeConstant().isEmpty(), "turned off, it is left in the disturbance");

    final ADRCConfig flywheel = new ADRCConfig().withSystemOrder(SystemOrder.FLYWHEEL);
    flywheel.buildController(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20));
    assertTrue(flywheel.getModeledBackEmfTimeConstant().isEmpty(), "a flywheel still lumps it into the disturbance");

    final ADRCConfig geared = new ADRCConfig().withSystemOrder(SystemOrder.PIVOT);
    geared.buildController(motor, smcConfig().withGearing(new MechanismGearing(GearBox.fromReductionStages(60)))
        .withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20));
    assertTrue(geared.getModeledBackEmfTimeConstant().isEmpty(),
        "a first order position loop's speed per volt already includes the back-EMF");

    final ADRCConfig fixed = new ADRCConfig().withSystemOrder(SystemOrder.SECOND).withB0(10);
    fixed.buildController(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20));
    assertTrue(fixed.getModeledBackEmfTimeConstant().isEmpty(), "a fixed order leaves the model to the user");

    final ADRCConfig noInertia = new ADRCConfig().withSystemOrder(SystemOrder.FLYWHEEL).withB0(10);
    noInertia.buildController(motor, smcConfig(), Milliseconds.of(20));
    assertTrue(noInertia.getModeledBackEmfTimeConstant().isEmpty(), "without an inertia there is nothing to model");
  }

  @Test
  void aTrackingDifferentiatorAndAMotionProfileAreRefused() {
    final DCMotor motor = DCMotor.getNEO(1);
    final ADRCConfig cfg = new ADRCConfig().withSystemOrder(SystemOrder.FIRST).withB0(10)
        .withTrackingDifferentiator(TrackingDifferentiator.Method.LD, 0.08);
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.buildController(motor, smcConfig().withTrapezoidalProfile(
            RotationsPerSecond.of(1), RotationsPerSecondPerSecond.of(2)), Milliseconds.of(20)),
        "a trapezoidal profile and a tracking differentiator both shape the setpoint");
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.buildController(motor, smcConfig().withExponentialProfile(Volts.of(12), motor,
            KilogramSquareMeters.of(0.1)), Milliseconds.of(20)),
        "an exponential profile and a tracking differentiator both shape the setpoint");
    cfg.buildController(motor, smcConfig(), Milliseconds.of(20));
    new ADRCConfig().withSystemOrder(SystemOrder.FIRST).withB0(10).buildController(motor,
        smcConfig().withTrapezoidalProfile(RotationsPerSecond.of(1), RotationsPerSecondPerSecond.of(2)),
        Milliseconds.of(20));
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
  void anElevatorIsFirstOrderWithItsSpeedPerVoltInMetersAsB0() {
    final DCMotor motor = DCMotor.getNEO(1);
    assertTrue(SystemOrder.ELEVATOR.isChosenWhenBuilt() && SystemOrder.ELEVATOR.requiresLinearClosedLoop());
    assertFalse(SystemOrder.FIRST.isChosenWhenBuilt());
    final ADRCConfig cfg = new ADRCConfig().withSystemOrder(SystemOrder.ELEVATOR);
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.resolveB0(motor, smcConfig().withMechanismCircumference(Meters.of(0.15))),
        "an elevator's closed loop must be linear");
    final yams.commands2.config.SmartMotorControllerConfig linear =
        smcConfig().withMechanismCircumference(Meters.of(0.15)).withLinearClosedLoopController(true);
    assertEquals(motor.Kv / (2 * Math.PI * 12) * 0.15, cfg.resolveB0(motor, linear), 1e-12,
        "without an inertia, the motor's free speed per volt through a 12:1 reduction and a 0.15 m circumference");
    cfg.withB0(3);
    assertEquals(3, cfg.resolveB0(motor, linear), "a b0 that is set wins");
    assertThrows(SmartMotorControllerConfigurationException.class,
        () -> cfg.resolveB0(motor, smcConfig()), "even with a b0 set, the closed loop must be linear");
  }

  /**
   * A position loop chosen when built: a 60:1 NEO pivot's back-EMF time constant, 6 ms, is under two
   * 20 ms periods, so it is first order with its speed per volt; at 12:1 it is 160 ms, so it is
   * second order with its acceleration per volt.
   */
  @Test
  void aPositionLoopIsChosenFromTheBackEmfTimeConstant() {
    final DCMotor motor = DCMotor.getNEO(1);
    for (SystemOrder order : new SystemOrder[] {SystemOrder.POSITION, SystemOrder.ARM, SystemOrder.PIVOT}) {
      assertTrue(order.isChosenWhenBuilt());
      assertEquals(0, order.getOrder());
      final yams.commands2.config.SmartMotorControllerConfig geared = smcConfig()
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(60)))
          .withMomentOfInertia(KilogramSquareMeters.of(0.1));
      final ADRCController first = new ADRCConfig().withSystemOrder(order)
          .buildController(motor, geared, Milliseconds.of(20));
      assertEquals(1, first.getSystemOrder(), order + " at 60:1");
      assertEquals(motor.Kv / (2 * Math.PI * 60), first.getB0(), 1e-9, order + " at 60:1: the speed per volt");
      final ADRCController second = new ADRCConfig().withSystemOrder(order)
          .buildController(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20));
      assertEquals(2, second.getSystemOrder(), order + " at 12:1");
      assertEquals(12 * motor.Kt / (2 * Math.PI * motor.R * 0.1), second.getB0(), 1e-9,
          order + " at 12:1: the acceleration per volt");
      assertThrows(SmartMotorControllerConfigurationException.class,
          () -> new ADRCConfig().withSystemOrder(order).buildController(motor, smcConfig(), Milliseconds.of(20)),
          order + " needs an inertia to choose");
      assertThrows(IllegalStateException.class, () -> new ADRCController(new ADRCConfig().withSystemOrder(order)),
          order + " is only chosen when built from a motor config");
    }
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
        perRotation * circumference, new ADRCConfig().withSystemOrder(SystemOrder.SECOND).resolveB0(motor, smc), 1e-9);
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
