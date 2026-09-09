// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers.simulation;

import static edu.wpi.first.units.Units.Amps;
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
        .withControlMode(ControlMode.OPEN_LOOP);

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
   * Schedule a heavy (near-stall) duty cycle command for the given {@link SmartMotorController}.
   */
  private static Command heavyLoadCommand(SmartMotorController smc)
  {
    return ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).setDutyCycle(1.0);
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
   */
  private static double minVoltageOverCycle(double seconds) throws InterruptedException
  {
    double[] minVoltage = {RoboRioSim.getVInVoltage()};
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
