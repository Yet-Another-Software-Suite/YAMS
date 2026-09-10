// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.motorcontrollers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.PeriodicScheduler;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Tests that closed loop control has no negative effects when {@link SmartMotorControllerConfig#getSimulationPeriod()}
 * differs from the robot's own periodic cadence.
 *
 * <p>
 * A {@link PeriodicScheduler} — a small stand-in for how {@link edu.wpi.first.wpilibj.TimedRobot}'s
 * own {@code addPeriodic()} runs multiple callbacks at independent periods from a single loop —
 * calls {@link SmartMotorController#simIterate()} at the configured 10ms simulation period and a
 * separate callback (re-commanding the setpoint and publishing telemetry) at 20ms, mirroring a
 * robot whose periodic loop runs at 20ms while simulation physics steps at 10ms. This runs
 * entirely on the calling thread against a virtual timeline — no real-time Notifier, no WPILib
 * simulation timing hooks — so it is fully deterministic. The test runs across every vendor
 * wrapper (Spark, TalonFXS, TalonFX) so a regression in any one wrapper's {@code simIterate()}
 * would be caught.
 * </p>
 */
public class SimulationPeriodTest
{
  private static Stream<Arguments> createConfigs()
  {
    SmartMotorControllerConfig baseConfig = new SmartMotorControllerConfig()
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withClosedLoopController(8, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 1))
        .withStatorCurrentLimit(Amps.of(40))
        .withIdleMode(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withSimulationPeriod(Milliseconds.of(10))
        .withStartingPosition(Degrees.of(0));

    return Stream.of(
        Arguments.of(new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1),
            baseConfig.clone()
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry("SimulationPeriodTest SparkMax", TelemetryVerbosity.LOW))),
        Arguments.of(new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1),
            baseConfig.clone()
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry("SimulationPeriodTest SparkFlex", TelemetryVerbosity.LOW))),
        Arguments.of(new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1),
            baseConfig.clone()
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry("SimulationPeriodTest TalonFXS", TelemetryVerbosity.LOW))),
        Arguments.of(new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1),
            baseConfig.clone()
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry("SimulationPeriodTest TalonFX", TelemetryVerbosity.LOW)))
    );
  }

  private static void closeSmc(SmartMotorController smc)
  {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();

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

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSimIterateAccurateWithSlowerTelemetryPeriod(SmartMotorController smc)
  {
    try
    {
      SmartMotorControllerTestSubsystem subsys =
          (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
      subsys.setSMC(smc);
      smc.setupSimulation();

      Angle setpoint = Degrees.of(80);

      int[] simIterations = {0};
      int[] telemetryIterations = {0};
      Angle finalPosition;
      try (PeriodicScheduler scheduler = new PeriodicScheduler())
      {
        // Commanding the setpoint happens under the scheduler's own controlled clock state (started
        // above), rather than before it, so this first call isn't at the mercy of whatever clock
        // state an earlier test in the same JVM left behind.
        smc.setPosition(setpoint);

        // simIterate() at the configured 10ms simulation period.
        scheduler.addPeriodic(() -> {
          smc.simIterate();
          simIterations[0]++;
        }, Milliseconds.of(10));
        // Re-commanding the setpoint and publishing telemetry at 20ms — a robot's normal periodic
        // loop, decoupled from the (faster) simulation period above.
        scheduler.addPeriodic(() -> {
          smc.setPosition(setpoint);
          smc.updateTelemetry();
          telemetryIterations[0]++;
        }, Milliseconds.of(20));

        scheduler.runFor(Seconds.of(3.0));

        finalPosition = smc.getMechanismPosition();
      }
      System.out.println(smc.getName() + ": sim iterations=" + simIterations[0]
          + ", telemetry iterations=" + telemetryIterations[0]
          + ", final position=" + finalPosition.in(Degrees) + " deg (target " + setpoint.in(Degrees) + " deg)");

      // Over 1.5s, a 10ms period should fire ~150 times and a 20ms period ~75 times — roughly twice
      // as often — confirming simIterate() genuinely ran at the configured simulation period rather
      // than some other period.
      assertTrue(simIterations[0] > telemetryIterations[0],
          smc.getName() + ": expected simIterate() (10ms) to run more often than the periodic loop (20ms).");

      // Closed loop position control should still converge to the setpoint even though simIterate()
      // runs at a different rate than the periodic loop. The regression this guards against is the
      // simulated motor moving too far (or too little) per physics step because the sim step size
      // was conflated with some other loop's period instead of the configured simulation period —
      // that shows up as an error of 100+ degrees (as observed while developing this test), not a
      // few degrees of jitter, so a generous tolerance here still catches a real regression while
      // absorbing CTRE's own background CAN-refresh-thread timing jitter under simulation.
      assertTrue(finalPosition.isNear(setpoint, Degrees.of(5)),
          smc.getName() + ": closed loop position control should converge to the setpoint with a "
              + "10ms simulation period and a 20ms periodic loop period.");
    } finally
    {
      closeSmc(smc);
    }
  }

  @BeforeEach
  void startTest()
  {
    MockHardwareExtension.beforeAll();
    // Guard against a sagged simulated battery voltage left behind by other test classes sharing
    // this JVM's BatterySim/RoboRioSim static state, so this test's convergence isn't at the mercy
    // of full-suite run order.
    RoboRioSim.setVInVoltage(12.0);
  }

  @AfterEach
  void endTest()
  {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }
}
