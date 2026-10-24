// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RPM;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Live tunes the LQR arm and flywheel over NetworkTables the way the YAMS Live Tuner does. One robot runs every
 * test: a second one would claim the same CAN IDs.
 */
class LQRLiveTuningTest {
  private static final double kLoopSeconds = 0.02;
  private static final String kArm = "/Tuning/LQRArm/ArmMotor/";
  private static final String kFlyWheel = "/Tuning/LQRFlyWheel/FlyWheelMotor/";
  private static final String kArmCommand = "/Tuning/LQRArmSubsystem/Live Tuning/running";
  private static final String kFlyWheelCommand = "/Tuning/LQRFlyWheelSubsystem/Live Tuning/running";

  private static final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private static Robot robot;
  private static Thread robotThread;

  @BeforeAll
  static void startRobot() {
    assertTrue(HAL.initialize());
    SimHooks.pauseTiming();
    DriverStationSim.resetData();
    robot = new Robot();
    robotThread = new Thread(robot::startCompetition);
    robotThread.start();
    SimHooks.waitForProgramStart();
    DriverStationSim.setDsAttached(true);
    DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
    DriverStationSim.setEnabled(true);
    DriverStationSim.notifyNewData();
    step(0.2);
  }

  @BeforeEach
  void stopLiveTuning() {
    set(kArmCommand, false);
    set(kFlyWheelCommand, false);
    CommandScheduler.getInstance().cancelAll();
    // Back to the configured LQRs, so no test sees another's tuning.
    set(kArm + "closedloop/lqr/qelms/position", 5.0);
    set(kArm + "closedloop/lqr/qelms/velocity", 10.0);
    set(kArm + "closedloop/lqr/encoder/position", 0.5);
    set(kFlyWheel + "closedloop/lqr/qelms/velocity", 100.0);
    step(0.1);
  }

  @AfterAll
  static void stopRobot() throws InterruptedException {
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void anLQRReplacesTheFeedbackGains() {
    for (String gain : new String[] {"kP", "kI", "kD"}) {
      assertFalse(nt.getTopic(kArm + "closedloop/feedback/" + gain).exists(), "the arm publishes no " + gain);
      assertFalse(nt.getTopic(kFlyWheel + "closedloop/feedback/" + gain).exists(), "the flywheel publishes no " + gain);
    }
    // An arm's LQR models position and velocity, and its encoder measures position.
    for (String field : new String[] {"qelms/position", "qelms/velocity", "relms", "model/position", "model/velocity", "encoder/position", "maxVoltage"}) {
      assertTrue(nt.getTopic(kArm + "closedloop/lqr/" + field).exists(), "the arm publishes lqr/" + field);
    }
    assertFalse(nt.getTopic(kArm + "closedloop/lqr/encoder/velocity").exists());
    // A flywheel's models only velocity.
    for (String field : new String[] {"qelms/velocity", "relms", "model/velocity", "encoder/velocity", "maxVoltage"}) {
      assertTrue(nt.getTopic(kFlyWheel + "closedloop/lqr/" + field).exists(), "the flywheel publishes lqr/" + field);
    }
    assertFalse(nt.getTopic(kFlyWheel + "closedloop/lqr/qelms/position").exists());
  }

  @Test
  void theFieldsStartAtTheConfiguredValuesInSetpointUnits() {
    // 5 degrees and 60 deg/s, published in degrees and RPM like the setpoints.
    assertEquals(5, nt.getEntry(kArm + "closedloop/lqr/qelms/position").getDouble(0), 1e-9);
    assertEquals(10, nt.getEntry(kArm + "closedloop/lqr/qelms/velocity").getDouble(0), 1e-9);
    assertEquals(100, nt.getEntry(kFlyWheel + "closedloop/lqr/qelms/velocity").getDouble(0), 1e-9);
    assertEquals(12, nt.getEntry(kFlyWheel + "closedloop/lqr/relms").getDouble(0), 1e-9);
  }

  @Test
  void theFlyWheelReachesItsSpeed() {
    CommandScheduler.getInstance().schedule(robot.flyWheel.spinUp(RPM.of(3000)));
    step(2);
    assertEquals(3000, robot.flyWheel.getSpeed().in(RPM), 150, "flywheel speed");
  }

  @Test
  void liveTuningRetunesTheFlyWheelsLQR() {
    set(kFlyWheel + "closedloop/setpoint/velocity", 2000.0);
    set(kFlyWheelCommand, true);
    step(2);
    assertEquals(2000, robot.flyWheel.getSpeed().in(RPM), 150, "flywheel speed while live tuning");
    set(kFlyWheel + "closedloop/lqr/qelms/velocity", 20.0);
    step(0.1);
    final double[] qelms = robot.flyWheel.getLQR().getConfig().orElseThrow().getQelms();
    assertArrayEquals(new double[] {RPM.of(20).in(org.wpilib.units.Units.RadiansPerSecond)}, qelms, 1e-9, "the LQR got the new tolerance");
    step(1);
    assertEquals(2000, robot.flyWheel.getSpeed().in(RPM), 150, "still at speed after the retune");
  }

  @Test
  void liveTuningRetunesTheArmsLQR() {
    set(kArm + "closedloop/setpoint/position", 30.0);
    set(kArmCommand, true);
    step(3);
    assertEquals(30, robot.arm.getAngle().in(Degrees), 3, "arm angle while live tuning");
    set(kArm + "closedloop/lqr/qelms/position", 1.0);
    set(kArm + "closedloop/lqr/encoder/position", 0.2);
    step(0.1);
    final var config = robot.arm.getLQR().getConfig().orElseThrow();
    assertEquals(Degrees.of(1).in(org.wpilib.units.Units.Radians), config.getQelms()[0], 1e-9, "the new position tolerance");
    assertEquals(Degrees.of(0.2).in(org.wpilib.units.Units.Radians), config.getEncoderStdDevs()[0], 1e-9, "the new encoder std dev");
    step(2);
    assertEquals(30, robot.arm.getAngle().in(Degrees), 3, "still holding after the retune");
  }

  @Test
  void aValueTheLQRCantUseLeavesItAsItWas() {
    set(kArmCommand, true);
    step(0.2);
    final double before = robot.arm.getLQR().getConfig().orElseThrow().getQelms()[1];
    set(kArm + "closedloop/lqr/qelms/velocity", 0.0);
    step(0.2);
    assertEquals(before, robot.arm.getLQR().getConfig().orElseThrow().getQelms()[1], 1e-12, "a tolerance of 0 is not applied");
    set(kArm + "closedloop/lqr/qelms/velocity", 10.0);
    step(0.1);
  }

  private static void set(String topic, Object value) {
    nt.getEntry(topic).setValue(value);
  }

  private static void step(double seconds) {
    for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
      SimHooks.stepTiming(kLoopSeconds);
    }
  }
}
