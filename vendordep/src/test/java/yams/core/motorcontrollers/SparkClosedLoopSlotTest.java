// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.function.DoubleSupplier;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.EnumSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests that each YAMS closed loop slot reaches the matching SPARK slot, and that the SPARK runs
 * velocity control from whichever slot is selected. Each slot gets different gains, so a slot mix-up
 * shows up as the wrong gains.
 */
public class SparkClosedLoopSlotTest {
  /** 12 motor rotations per mechanism rotation. */
  private static final MechanismGearing kGearing =
      new MechanismGearing(GearBox.fromReductionStages(3, 4));

  private static final double kRatio = 12;

  /** A NEO's kV through 12:1, in volts per mechanism rotation per second: 12 / (473 RPM/V / 60). */
  private static final double kV = kRatio / (473.0 / 60);

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** kP for a slot: 0.1, 0.2, 0.3 and 0.4 volts per mechanism rotation, so each slot differs. */
  private static double kP(ClosedLoopControllerSlot slot) {
    return 0.1 * (slot.ordinal() + 1);
  }

  /** kS for a slot: 0.05, 0.1, 0.15 and 0.2 volts. */
  private static double kS(ClosedLoopControllerSlot slot) {
    return 0.05 * (slot.ordinal() + 1);
  }

  private static ClosedLoopSlot sparkSlot(ClosedLoopControllerSlot slot) {
    return ClosedLoopSlot.values()[slot.ordinal()];
  }

  /** A config with every slot filled in, each with its own gains. */
  private static SmartMotorControllerConfig config(String name) {
    final SmartMotorControllerConfig config =
        new yams.commands2.config.SmartMotorControllerConfig()
            .withSubsystem(new SmartMotorControllerTestSubsystem())
            .withGearing(kGearing)
            .withStatorCurrentLimit(Amps.of(40))
            .withZeroPower(MotorMode.COAST)
            .withControlMode(ControlMode.CLOSED_LOOP)
            .withMomentOfInertia(KilogramSquareMeters.of(0.002))
            .withTelemetry(
                "SparkClosedLoopSlotTest " + name,
                yams.core.telemetry.enums.TelemetryVerbosity.LOW);
    for (ClosedLoopControllerSlot slot : ClosedLoopControllerSlot.values()) {
      config
          .withClosedLoopController(kP(slot), 0, 0, slot)
          .withFeedforward(new SimpleMotorFeedforward(kS(slot), kV, 0), slot);
    }
    return config;
  }

  private static SmartMotorController create(SparkMax sparkMax, String name) {
    final SmartMotorController smc =
        new SparkWrapper(sparkMax, DCMotor.getNEO(1), config(name));
    ((SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem())
        .setSMC(smc);
    smc.setupSimulation();
    return smc;
  }

  private static void close(SmartMotorController smc, SparkMax sparkMax) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);
    sparkMax.configure(
        new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    sparkMax.close();
  }

  /** Wait for an asynchronously applied SPARK parameter to reach the expected value. */
  private static void assertDevice(double expected, DoubleSupplier actual, String what)
      throws InterruptedException {
    assertTrue(
        AbsoluteEncoderCases.eventually(
            () ->
                Math.abs(actual.getAsDouble() - expected)
                    <= 1e-4 * Math.max(1, Math.abs(expected))),
        what + ": expected " + expected + " on the SPARK but was " + actual.getAsDouble());
  }

  /**
   * Each YAMS slot's gains are written to the SPARK slot of the same number, in the SPARK's units:
   * kP per rotation of its encoder at a 12 V scale, kS in volts, and kV per motor RPM.
   */
  @ParameterizedTest
  @EnumSource(ClosedLoopControllerSlot.class)
  void gainsReachTheMatchingSparkSlot(ClosedLoopControllerSlot slot) throws InterruptedException {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc = create(sparkMax, "gains " + slot);
    try {
      final ClosedLoopSlot sparkSlot = sparkSlot(slot);
      final var closedLoop = sparkMax.configAccessor.closedLoop;
      assertDevice(kP(slot) / (12 * kRatio), () -> closedLoop.getP(sparkSlot), slot + " kP");
      assertDevice(kS(slot), () -> closedLoop.feedForward.getkS(sparkSlot), slot + " kS");
      assertDevice(
          kV / RPM.of(kRatio * 60).in(RPM), () -> closedLoop.feedForward.getkV(sparkSlot), slot + " kV");
    } finally {
      close(smc, sparkMax);
    }
  }

  /**
   * With a slot selected, velocity control reaches its setpoint in simulation. The feedforward does
   * nearly all the work, so this fails if the selected slot's kV is not applied.
   *
   * <p>Disabled until REVLib's simulated SPARK applies kV outside slot 0: in REVLib
   * 2027.0.0-alpha-7, SLOT_1 to SLOT_3 only reach 16 to 27 of 100 RPM. SLOT_0 passes.
   */
  @Disabled("REVLib 2027.0.0-alpha-7's simulated SPARK only applies kV in slot 0; "
      + "see https://github.com/wpilibsuite/SystemcoreTesting/issues/425")
  @ParameterizedTest
  @EnumSource(ClosedLoopControllerSlot.class)
  void velocityControlUsesTheSelectedSlot(ClosedLoopControllerSlot slot) {
    final SparkMax sparkMax = DeviceCreator.createSparkMax();
    final SmartMotorController smc = create(sparkMax, "velocity " + slot);
    try {
      smc.setClosedLoopSlot(slot);
      // 100 mechanism RPM is 1200 motor RPM, about 2.5 V of kV.
      smc.setVelocity(RPM.of(100));
      for (int i = 0; i < 100; i++) {
        smc.simIterate();
      }
      final double rpm = smc.getMechanismVelocity().in(RPM);
      System.out.println(slot + ": " + rpm + " mechanism RPM, " + smc.getMechanismPosition().in(Rotations) + " rotations");
      assertEquals(100, rpm, 5, slot + " velocity control should reach 100 RPM");
    } finally {
      close(smc, sparkMax);
    }
  }
}
