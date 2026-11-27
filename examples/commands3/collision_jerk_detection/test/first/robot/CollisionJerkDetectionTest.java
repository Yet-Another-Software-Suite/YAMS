// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.BooleanSubscriber;
import org.wpilib.networktables.IntegerSubscriber;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

class CollisionJerkDetectionTest {
  private static final double LOOP_SECONDS = 0.02;

  private Robot robot;
  private Thread robotThread;
  private BooleanSubscriber bumpedSub;
  private IntegerSubscriber collisionCountSub;

  @BeforeEach
  void startRobot() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.resetData();

    robot = new Robot();
    robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();

    var nt = NetworkTableInstance.getDefault();
    bumpedSub = nt.getBooleanTopic("/Collision/BumpedRecently").subscribe(true);
    collisionCountSub = nt.getIntegerTopic("/Collision/CollisionCount").subscribe(-1L);

    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.4);
  }

  @AfterEach
  void stopRobot() throws InterruptedException {
    bumpedSub.close();
    collisionCountSub.close();
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
  void publishesZeroCollisionsWhenDrivingCleanlyInSimulation() {
    step(0.5);
    assertFalse(bumpedSub.get());
    assertEquals(0L, collisionCountSub.get());
  }
}
