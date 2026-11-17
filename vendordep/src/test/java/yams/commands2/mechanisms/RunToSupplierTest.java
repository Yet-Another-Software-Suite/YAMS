// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkMax;
import java.util.concurrent.atomic.AtomicReference;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * {@code runTo(Supplier, tolerance)} must follow the supplier while it runs, and end only once the
 * mechanism is near the supplier's current value, not the value it had when the command was built.
 */
public class RunToSupplierTest {
  private SparkMax spark;
  private SmartMotorController smc;
  private SmartMotorControllerTestSubsystem subsystem;
  private Pivot pivot;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
    spark = DeviceCreator.createSparkMax();
    subsystem = new SmartMotorControllerTestSubsystem();
    SmartMotorControllerConfig config = new SmartMotorControllerConfig()
        .withSubsystem(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withClosedLoopController(20, 0, 0.5)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSoftLimits(Degrees.of(-100), Degrees.of(100))
        .withStartingPosition(Degrees.of(0))
        .withMomentOfInertia(Inches.of(4), Pounds.of(1))
        .withTelemetry("RunToSupplierPivot", TelemetryVerbosity.LOW);
    smc = new SparkWrapper(spark, DCMotor.getNEO(1), config);
    subsystem.smc = smc;
    pivot = new Pivot(new PivotConfig().withHardLimits(Degrees.of(-150), Degrees.of(150)), smc);
  }

  @AfterEach
  void endTest() {
    TestWithScheduler.schedulerClear();
    SmartMotorControllerCommandRegistry.removeCommands(subsystem);
    CommandScheduler.getInstance().unregisterSubsystem(subsystem);
    subsystem.close();
    DeviceCreator.silence(smc);
    spark.close();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  @Test
  void followsATargetThatChangesWhileItRuns() {
    AtomicReference<Angle> target = new AtomicReference<>(Degrees.of(30));
    Command command = pivot.runTo(target::get, Degrees.of(3));
    TestWithScheduler.schedule(command);

    // Change the target before the pivot can reach the first one.
    TestWithScheduler.cycle(Seconds.of(0.04), pivot::simIterate);
    target.set(Degrees.of(60));

    TestWithScheduler.cycle(Seconds.of(4), () -> {
      pivot.simIterate();
      if (!command.isScheduled()) {
        throw new IllegalStateException("done");
      }
    });

    assertFalse(command.isScheduled(), "runTo should end once the pivot reaches the new target.");
    assertEquals(60, pivot.getAngle().in(Degrees), 3,
                 "runTo should have followed the supplier to 60 degrees, not stopped at 30.");
  }

  @Test
  void keepsRunningUntilItReachesTheTarget() {
    AtomicReference<Angle> target = new AtomicReference<>(Degrees.of(45));
    Command command = pivot.runTo(target::get, Degrees.of(3));
    TestWithScheduler.schedule(command);
    TestWithScheduler.cycle(Seconds.of(0.04), pivot::simIterate);
    assertTrue(command.isScheduled(), "runTo should not end before the pivot gets near the target.");
  }
}
