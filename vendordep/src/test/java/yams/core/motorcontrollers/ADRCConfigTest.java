// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Volts;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCController;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.config.ADRCConfig;
import yams.helpers.MockHardwareExtension;

/**
 * {@link ADRCConfig}: reading {@code deploy/adrc/<name>.json}, the Java values that override it,
 * the {@code b0} it derives, and how {@code withADRCController} replaces the other closed loop
 * controllers and feedforwards of its slot.
 */
class ADRCConfigTest {
  @TempDir
  Path deploy;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
  }

  private void write(String name, String json) throws IOException {
    Files.createDirectories(deploy.resolve("adrc"));
    Files.writeString(deploy.resolve("adrc").resolve(name + ".json"), json);
  }

  private static yams.commands2.config.SmartMotorControllerConfig smcConfig() {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(12)));
  }

  @Test
  void readsEveryKeyFromItsFile() throws IOException {
    write("Shooter", """
        {
          "systemOrder": 2,
          "settlingTime": 0.4,
          "bandwidthScale": 1.5,
          "b0": 42.0,
          "minVoltage": -6.0,
          "maxVoltage": 10.0,
          "slewRateLimit": 24.0,
          "inputDelay": 0.04,
          "cascaded": true,
          "tdMethod": "LD",
          "tdParams": [0.5]
        }
        """);
    final ADRCConfig cfg = new ADRCConfig("Shooter", deploy);
    assertEquals(2, cfg.getSystemOrder().orElseThrow());
    assertEquals(0.4, cfg.getSettlingTime());
    assertEquals(1.5, cfg.getBandwidthScale());
    assertEquals(42.0, cfg.getB0().orElseThrow());
    assertEquals(-6.0, cfg.getMinVoltage().orElseThrow());
    assertEquals(10.0, cfg.getMaxVoltage().orElseThrow());
    assertEquals(24.0, cfg.getSlewRateLimit());
    assertEquals(0.04, cfg.getInputDelay());
    assertTrue(cfg.isCascaded());
    assertEquals("ld", cfg.getTdMethod());
    assertArrayEquals(new double[] {0.5}, cfg.getTdParams());
  }

  @Test
  void javaValuesOverrideTheFile() throws IOException {
    write("Arm", "{\"systemOrder\": 1, \"settlingTime\": 0.4, \"b0\": 3.0}");
    final ADRCConfig cfg = new ADRCConfig("Arm", deploy).withSystemOrder(2).withB0(5.0);
    assertEquals(2, cfg.getSystemOrder().orElseThrow());
    assertEquals(5.0, cfg.getB0().orElseThrow());
    assertEquals(0.4, cfg.getSettlingTime(), "a value Java leaves alone comes from the file");
  }

  @Test
  void aMissingFileLeavesTheDefaultsAndNoSystemOrder() {
    final ADRCConfig cfg = new ADRCConfig("Missing", deploy);
    assertTrue(cfg.getSystemOrder().isEmpty());
    assertEquals(ADRCConfig.DEFAULT_SETTLING_TIME, cfg.getSettlingTime());
    assertEquals(ADRCConfig.DEFAULT_BANDWIDTH_SCALE, cfg.getBandwidthScale());
    assertEquals(ADRCConfig.DEFAULT_TD_METHOD, cfg.getTdMethod());
    assertFalse(cfg.isCascaded());
    assertThrows(
        SmartMotorControllerConfigurationException.class,
        () -> cfg.buildController(DCMotor.getNEO(1), smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1)), Milliseconds.of(20)),
        "there is no default system order");
  }

  @Test
  void aBadValueIsSkippedAndTheRestApplied() throws IOException {
    write("Elevator", """
        {
          "systemOrder": 1.5,
          "settlingTime": "fast",
          "bandwidthScale": -1,
          "b0": 0,
          "cascaded": "yes",
          "tdMethod": "magic",
          "tdParams": [1, "two"],
          "maxVoltage": 8.0,
          "somethingElse": 3
        }
        """);
    final ADRCConfig cfg = new ADRCConfig("Elevator", deploy);
    assertTrue(cfg.getSystemOrder().isEmpty());
    assertEquals(ADRCConfig.DEFAULT_SETTLING_TIME, cfg.getSettlingTime());
    assertEquals(ADRCConfig.DEFAULT_BANDWIDTH_SCALE, cfg.getBandwidthScale());
    assertTrue(cfg.getB0().isEmpty());
    assertFalse(cfg.isCascaded());
    assertEquals(ADRCConfig.DEFAULT_TD_METHOD, cfg.getTdMethod());
    assertNull(cfg.getTdParams());
    assertEquals(8.0, cfg.getMaxVoltage().orElseThrow(), "a good value next to bad ones applies");
  }

  @Test
  void limitsThatCrossAreBothSkipped() throws IOException {
    write("Crossed", "{\"minVoltage\": 6.0, \"maxVoltage\": 4.0}");
    final ADRCConfig cfg = new ADRCConfig("Crossed", deploy);
    assertTrue(cfg.getMinVoltage().isEmpty());
    assertTrue(cfg.getMaxVoltage().isEmpty());
  }

  @Test
  void aFileThatIsNotJsonLeavesTheDefaults() throws IOException {
    write("Broken", "{\"systemOrder\": 1,");
    assertTrue(new ADRCConfig("Broken", deploy).getSystemOrder().isEmpty());
    write("Array", "[1, 2]");
    assertTrue(new ADRCConfig("Array", deploy).getSystemOrder().isEmpty());
  }

  @Test
  void rejectsANameThatIsNotAFileName() {
    assertThrows(IllegalArgumentException.class, () -> new ADRCConfig("", deploy));
    assertThrows(IllegalArgumentException.class, () -> new ADRCConfig("../Shooter", deploy));
    assertThrows(IllegalArgumentException.class, () -> new ADRCConfig("adrc/Shooter", deploy));
  }

  @Test
  void derivesB0FromTheMotorGearingAndInertia() {
    final DCMotor motor = DCMotor.getNEO(1);
    final ADRCConfig cfg = new ADRCConfig("Derived", deploy).withSystemOrder(2);
    assertThrows(
        SmartMotorControllerConfigurationException.class,
        () -> cfg.resolveB0(motor, smcConfig()),
        "the placeholder inertia is never used");
    final double expected = 12 * motor.Kt / (2 * Math.PI * motor.R * 0.1);
    assertEquals(expected, cfg.resolveB0(motor, smcConfig().withMomentOfInertia(KilogramSquareMeters.of(0.1))), 1e-9);
    assertEquals(7.0, cfg.withB0(7.0).resolveB0(motor, smcConfig()), "a b0 that is set wins");
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
    assertEquals(perRotation * circumference, new ADRCConfig("Linear", deploy).resolveB0(motor, smc), 1e-9);
  }

  @Test
  void buildsAControllerFromTheConfig() {
    final ADRCController controller =
        new ADRCConfig("Built", deploy)
            .withSystemOrder(2)
            .withSettlingTime(0.5)
            .withB0(10)
            .buildController(
                DCMotor.getNEO(1),
                smcConfig().withClosedLoopControllerMaximumVoltage(Volts.of(9)),
                Milliseconds.of(10));
    assertEquals(2, controller.getSystemOrder());
    assertEquals(0.5, controller.getSettlingTime());
    assertEquals(10, controller.getB0());
    assertEquals(9, controller.getMaxOutput(), "the motor config's closed loop voltage limit");
    assertEquals(-9, controller.getMinOutput());
    final ADRCController fallback =
        new ADRCConfig("Fallback", deploy).withSystemOrder(1).withB0(10).buildController(DCMotor.getNEO(1), smcConfig(), Milliseconds.of(20));
    assertEquals(DCMotor.getNEO(1).nominalVoltage, fallback.getMaxOutput(), "the motor's nominal voltage");
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
            .withADRCController(new ADRCConfig("Replace", deploy).withSystemOrder(1));
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
            .withADRCController(new ADRCConfig("Slot0", deploy).withSystemOrder(1), ClosedLoopControllerSlot.SLOT_0);
    assertTrue(smc.getADRCController(ClosedLoopControllerSlot.SLOT_0).isPresent());
    assertTrue(smc.getPID(ClosedLoopControllerSlot.SLOT_1).isPresent());
    assertTrue(smc.getADRCController(ClosedLoopControllerSlot.SLOT_1).isEmpty());
  }

  @Test
  void aCopyOfTheMotorConfigGetsItsOwnADRCConfig() {
    final yams.commands2.config.SmartMotorControllerConfig smc =
        smcConfig().withADRCController(new ADRCConfig("Copied", deploy).withSystemOrder(1).withB0(3));
    final yams.commands2.config.SmartMotorControllerConfig copy =
        (yams.commands2.config.SmartMotorControllerConfig) smc.clone();
    assertNotSame(smc.getADRCController().orElseThrow(), copy.getADRCController().orElseThrow());
    copy.getADRCController().orElseThrow().withB0(9);
    assertEquals(3, smc.getADRCController().orElseThrow().getB0().orElseThrow(), "changing the copy leaves the original");
  }
}
