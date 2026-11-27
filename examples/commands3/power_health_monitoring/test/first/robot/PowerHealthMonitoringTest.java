// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.PowerHealthMonitor.PowerState;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.StringSubscriber;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SimHooks;
import yams.core.motorcontrollers.simulation.BatterySim;

class PowerHealthMonitoringTest {
  private static final double LOOP_SECONDS = 0.02;

  private Robot robot;
  private Thread robotThread;
  private DoubleSubscriber throttleScaleSub;
  private IntegerSubscriber brownoutCountSub;
  private StringSubscriber stateSub;

  @BeforeEach
  void startRobot() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    BatterySim.resetDischarge();
    RoboRioSim.setVInVoltage(12.4);
    DriverStationSim.resetData();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();

    var nt = NetworkTableInstance.getDefault();
    throttleScaleSub = nt.getDoubleTopic("/PowerHealth/ThrottleScale").subscribe(0.0);
    brownoutCountSub = nt.getIntegerTopic("/PowerHealth/BrownoutCount").subscribe(-1L);
    stateSub = nt.getStringTopic("/PowerHealth/State").subscribe("");

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.4);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    throttleScaleSub.close();
    brownoutCountSub.close();
    stateSub.close();
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  private void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / LOOP_SECONDS); i++) {
      SimHooks.stepTiming(LOOP_SECONDS);
    }
  }

  @Test
  void publishesNominalPowerHealthInSimulation() {
    step(0.5);
    assertEquals(PowerState.NOMINAL.name(), stateSub.get());
    assertEquals(1.0, throttleScaleSub.get(), 1e-6);
    assertEquals(0L, brownoutCountSub.get());
  }
}