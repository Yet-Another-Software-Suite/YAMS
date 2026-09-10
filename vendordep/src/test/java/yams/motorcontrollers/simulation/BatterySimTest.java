// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.simulation;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.MilliOhms;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerCommandRegistry;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Tests that {@link BatterySim} models the shared simulated battery sagging and draining as
 * multiple {@link SmartMotorController}s draw heavy current simultaneously, mirroring what happens
 * on a real robot when several mechanisms are commanded at once.
 */
public class BatterySimTest
{
  private static int offset = 0;

  /**
   * Allowed slack when asserting that voltage does not rise, to absorb floating point/timing
   * noise without masking a real regression (which showed up as multi-volt jumps).
   */
  private static final double VOLTAGE_RISE_TOLERANCE = 0.05;

  /**
   * Build four heavily-loadable {@link SmartMotorController}s (one per supported vendor wrapper),
   * each with its own {@link SmartMotorControllerTestSubsystem} and simulation set up, ready to be
   * commanded to a heavy duty cycle.
   */
  private static List<SmartMotorController> createHeavyLoadSmcs()
  {
    offset += 1;
    SmartMotorControllerConfig baseConfig = new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.COAST)
        .withStatorCurrentLimit(Amps.of(80))
        .withMotorInverted(false)
        .withControlMode(ControlMode.OPEN_LOOP)
        // A heavy load (well above the 0.02 kgm^2 default) keeps the motor accelerating slowly, so
        // it stays near its high-current stall region for the whole test window instead of
        // quickly spinning up towards free speed and current dropping off — a stable, heavily
        // loaded mechanism rather than a fast, lightly loaded one.
        .withMomentOfInertia(KilogramSquareMeters.of(2.0));

    List<SmartMotorController> smcs = new ArrayList<>();
    smcs.add(new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
        baseConfig.clone()
                  .withSubsystem(new SmartMotorControllerTestSubsystem())
                  .withTelemetry("BatterySim SparkMax(" + (10 + offset) + ") NEO", TelemetryVerbosity.LOW)));
    smcs.add(new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1),
        baseConfig.clone()
                  .withSubsystem(new SmartMotorControllerTestSubsystem())
                  .withTelemetry("BatterySim SparkFlex(" + (20 + offset) + ") Vortex", TelemetryVerbosity.LOW)));
    smcs.add(new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1),
        baseConfig.clone()
                  .withSubsystem(new SmartMotorControllerTestSubsystem())
                  .withTelemetry("BatterySim TalonFXS(" + (30 + offset) + ") NEO", TelemetryVerbosity.LOW)));
    smcs.add(new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1),
        baseConfig.clone()
                  .withSubsystem(new SmartMotorControllerTestSubsystem())
                  .withTelemetry("BatterySim TalonFX(" + (40 + offset) + ") Kraken", TelemetryVerbosity.LOW)));

    for (SmartMotorController smc : smcs)
    {
      SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
      subsys.setSMC(smc);
      smc.setupSimulation();
      subsys.testRunning = true;
    }
    return smcs;
  }

  /**
   * Build a duty cycle command for the given {@link SmartMotorController}.
   */
  private static Command dutyCycleCommand(SmartMotorController smc, double dutyCycle)
  {
    return ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).setDutyCycle(dutyCycle);
  }

  /**
   * Schedule a heavy (near-stall) duty cycle command for the given {@link SmartMotorController}.
   */
  private static Command heavyLoadCommand(SmartMotorController smc)
  {
    return dutyCycleCommand(smc, 1.0);
  }

  /**
   * Force the simulated battery to a fully depleted state by enabling discharge with a
   * vanishingly small capacity and running a brief heavy load — any nonzero current instantly
   * exhausts it. Returns the {@link SmartMotorController}s used to do it, still under heavy load,
   * so the caller can drive further scenarios (e.g. idling or continuing to load a dead battery).
   */
  private static List<SmartMotorController> drainBatteryToEmpty() throws InterruptedException
  {
    BatterySim.enableDischarge(1e-6, Volts.of(12.0), MilliOhms.of(20));
    List<SmartMotorController> smcs = createHeavyLoadSmcs();
    for (SmartMotorController smc : smcs)
    {
      TestWithScheduler.schedule(heavyLoadCommand(smc));
    }
    TestWithScheduler.cycle(Seconds.of(0.1));
    return smcs;
  }

  /**
   * Close an {@link SmartMotorController} and its underlying vendor device, freeing HAL handles and
   * scheduler registrations.
   */
  private static void closeSmc(SmartMotorController smc)
  {
    SmartMotorControllerCommandRegistry.removeCommands(smc.getConfig().getSubsystem());
    CommandScheduler.getInstance().unregisterSubsystem(
        (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem());
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).close();

    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax)
    {
      ((SparkMax) motorController).close();
    } else if (motorController instanceof SparkFlex)
    {
      ((SparkFlex) motorController).close();
    } else if (motorController instanceof TalonFXS)
    {
      ((TalonFXS) motorController).close();
    } else if (motorController instanceof TalonFX)
    {
      ((TalonFX) motorController).close();
    }
  }

  /**
   * Track the lowest {@link RoboRioSim#getVInVoltage()} seen while cycling the scheduler for the
   * given duration. Each registered subsystem's {@code simulationPeriodic()} updates the shared
   * battery voltage sequentially within a tick, so a single instantaneous read after the fact is
   * noisy; the minimum over the whole window reliably captures the worst-case sag instead.
   *
   * @implNote Seeded with {@link Double#POSITIVE_INFINITY}, not the pre-cycle voltage — the latter
   * can be a stale reading from before this window's simulation ticks ran (e.g. the raw HAL default
   * of 12V, which is below a fully-charged battery's true ~12.9V open circuit voltage), which would
   * otherwise mask the real value for this window and make a later window look like a false rise.
   */
  private static double minVoltageOverCycle(double seconds) throws InterruptedException
  {
    double[] minVoltage = {Double.POSITIVE_INFINITY};
    TestWithScheduler.cycle(Seconds.of(seconds),
        () -> minVoltage[0] = Math.min(minVoltage[0], RoboRioSim.getVInVoltage()));
    return minVoltage[0];
  }

  @Test
  void testHeavyLoadFromMultipleSmcsSagsBatteryVoltage() throws InterruptedException
  {
    List<SmartMotorController> smcs = createHeavyLoadSmcs();
    try
    {
      // Let the sim settle at idle (zero duty cycle) and record the baseline voltage.
      double idleVoltage = minVoltageOverCycle(0.5);

      // Command a single motor to a heavy duty cycle and measure the resulting sag.
      TestWithScheduler.schedule(heavyLoadCommand(smcs.get(0)));
      double singleMotorVoltage = minVoltageOverCycle(0.3);

      // Now command every remaining motor to a heavy duty cycle at the same time.
      for (int i = 1; i < smcs.size(); i++)
      {
        TestWithScheduler.schedule(heavyLoadCommand(smcs.get(i)));
      }
      double allMotorsVoltage = minVoltageOverCycle(0.3);

      System.out.println("Idle voltage: " + idleVoltage);
      System.out.println("Single motor heavy load voltage: " + singleMotorVoltage);
      System.out.println("All motors heavy load voltage: " + allMotorsVoltage);

      assertTrue(singleMotorVoltage < idleVoltage,
          "Voltage should sag below idle once a single motor is drawing heavy current.");
      assertTrue(allMotorsVoltage < singleMotorVoltage,
          "Voltage should sag further once every motor is drawing heavy current simultaneously.");
    } finally
    {
      smcs.forEach(BatterySimTest::closeSmc);
    }
  }

  @Test
  void testSustainedHeavyLoadDrainsBatteryStateOfCharge() throws InterruptedException
  {
    // Use a small capacity so a couple of simulated seconds of heavy current draw is a measurable
    // fraction of the battery, without needing to simulate a realistic multi-hour match.
    // @implNote BatterySim's current-draw bookkeeping is process-global and never cleared per SMC,
    // so other test classes' leftover current draws can still be contributing when this runs as
    // part of the full suite; only assert the direction of the drain (strictly below full), not a
    // specific magnitude, so the test stays reliable regardless of run order.
    BatterySim.enableDischarge(2.0, Volts.of(12.0), MilliOhms.of(20));
    List<SmartMotorController> smcs = createHeavyLoadSmcs();
    try
    {
      assertTrue(BatterySim.getStateOfCharge() == 1.0, "Battery should start fully charged.");

      for (SmartMotorController smc : smcs)
      {
        TestWithScheduler.schedule(heavyLoadCommand(smc));
      }
      TestWithScheduler.cycle(Seconds.of(0.2));

      double stateOfCharge = BatterySim.getStateOfCharge();
      System.out.println("State of charge after sustained heavy load: " + stateOfCharge);
      assertTrue(stateOfCharge < 1.0,
          "Sustained heavy current draw from multiple motors should drain the battery's state of "
              + "charge.");
    } finally
    {
      smcs.forEach(BatterySimTest::closeSmc);
      BatterySim.disableDischarge();
    }
  }

  @Test
  void testFullyChargedBatteryVoltage() throws InterruptedException
  {
    BatterySim.enableDischarge(2.0, Volts.of(12.0), MilliOhms.of(20));
    List<SmartMotorController> smcs = createHeavyLoadSmcs();
    try
    {
      assertTrue(BatterySim.getStateOfCharge() == 1.0, "Battery should start fully charged.");

      // A fresh, fully charged battery sits at the top of BatterySim's discharge curve, above the
      // flat 12 V used when discharge simulation is disabled.
      double idleVoltage = minVoltageOverCycle(0.2);
      System.out.println("Fully charged idle voltage: " + idleVoltage);
      assertTrue(idleVoltage > 12.0,
          "A fully charged, unloaded battery should read above nominal 12V.");

      for (SmartMotorController smc : smcs)
      {
        TestWithScheduler.schedule(heavyLoadCommand(smc));
      }
      double loadedVoltage = minVoltageOverCycle(0.2);
      System.out.println("Fully charged voltage under heavy load: " + loadedVoltage);

      assertTrue(loadedVoltage < idleVoltage,
          "Voltage should sag from nominal once multiple motors draw heavy current, even on a full battery.");
      assertTrue(BatterySim.getStateOfCharge() < 1.0,
          "Drawing current from a fully charged battery should begin depleting its state of charge.");
    } finally
    {
      smcs.forEach(BatterySimTest::closeSmc);
      BatterySim.disableDischarge();
    }
  }

  @Test
  void testEmptyBatteryVoltage() throws InterruptedException
  {
    List<SmartMotorController> smcs = drainBatteryToEmpty();
    try
    {
      assertTrue(BatterySim.getStateOfCharge() == 0.0, "Battery should be fully depleted.");

      // Stop drawing current and let the depleted battery settle towards its empty open-circuit
      // voltage, which BatterySim's discharge curve puts well below a healthy battery's ~12 V.
      for (SmartMotorController smc : smcs)
      {
        TestWithScheduler.schedule(dutyCycleCommand(smc, 0.0));
      }
      double deadIdleVoltage = minVoltageOverCycle(0.2);
      System.out.println("Depleted battery idle voltage: " + deadIdleVoltage);
      assertTrue(deadIdleVoltage < 10.0,
          "A fully depleted battery should sag well below a healthy battery's nominal voltage.");
    } finally
    {
      smcs.forEach(BatterySimTest::closeSmc);
      BatterySim.disableDischarge();
    }
  }

  @Test
  void testVoltageDoesNotRiseWithMoreSmcsUnderHeavyLoad() throws InterruptedException
  {
    // Compare a single motor alone against every motor at once, each starting from an equally
    // fresh battery. (Incrementally adding motors one at a time across staggered time windows was
    // tried first and is not a reliable comparison: motor stall current decays as it spins up, so
    // the worst instantaneous demand doesn't necessarily grow in lockstep with motor count over
    // time — that's a real transient-dynamics property, not something worth asserting on.)
    BatterySim.enableDischarge(2.0, Volts.of(12.0), MilliOhms.of(20));
    List<SmartMotorController> soloSmcs = createHeavyLoadSmcs();
    double soloVoltage;
    try
    {
      TestWithScheduler.schedule(heavyLoadCommand(soloSmcs.get(0)));
      soloVoltage = minVoltageOverCycle(0.3);
    } finally
    {
      // Cancel the still-running heavy-load command before closing its device — otherwise the
      // scheduler tries to execute it against an already-closed SMC on the next cycle.
      TestWithScheduler.schedulerClear();
      soloSmcs.forEach(BatterySimTest::closeSmc);
    }

    BatterySim.resetDischarge();
    List<SmartMotorController> manySmcs = createHeavyLoadSmcs();
    double manyVoltage;
    try
    {
      for (SmartMotorController smc : manySmcs)
      {
        TestWithScheduler.schedule(heavyLoadCommand(smc));
      }
      manyVoltage = minVoltageOverCycle(0.3);
    } finally
    {
      manySmcs.forEach(BatterySimTest::closeSmc);
      BatterySim.disableDischarge();
    }

    System.out.println("Single motor heavy load voltage: " + soloVoltage);
    System.out.println("Many motors heavy load voltage: " + manyVoltage);
    assertTrue(manyVoltage <= soloVoltage + VOLTAGE_RISE_TOLERANCE,
        "An SMC's available voltage should not be higher when many other SMCs are also drawing "
            + "heavy current than when it is drawing heavy current alone.");
  }

  @Test
  void testVoltageDoesNotRiseWhenBatteryIsDead() throws InterruptedException
  {
    List<SmartMotorController> smcs = drainBatteryToEmpty();
    try
    {
      assertTrue(BatterySim.getStateOfCharge() == 0.0, "Battery should be fully depleted.");

      // Let the depleted battery idle and record its baseline voltage.
      for (SmartMotorController smc : smcs)
      {
        TestWithScheduler.schedule(dutyCycleCommand(smc, 0.0));
      }
      double deadIdleVoltage = minVoltageOverCycle(0.2);

      // The battery is already empty; hammering every motor with a heavy duty cycle again must not
      // make the reported voltage climb back up.
      for (SmartMotorController smc : smcs)
      {
        TestWithScheduler.schedule(heavyLoadCommand(smc));
      }
      double deadLoadedVoltage = minVoltageOverCycle(0.2);

      System.out.println("Dead battery idle voltage: " + deadIdleVoltage);
      System.out.println("Dead battery voltage under heavy load: " + deadLoadedVoltage);
      assertTrue(deadLoadedVoltage <= deadIdleVoltage + VOLTAGE_RISE_TOLERANCE,
          "Voltage should not rise above the depleted-battery baseline just because motors are drawing heavy "
              + "current.");
    } finally
    {
      smcs.forEach(BatterySimTest::closeSmc);
      BatterySim.disableDischarge();
    }
  }

  @BeforeEach
  void startTest()
  {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
    BatterySim.disableDischarge();
    BatterySim.resetDischarge();
  }

  @AfterEach
  void endTest()
  {
    BatterySim.disableDischarge();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
