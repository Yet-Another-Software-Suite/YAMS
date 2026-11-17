// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Drives the ADRC arm, elevator and shooter to their setpoints in simulation, and live tunes the arm's ADRC over
 * NetworkTables the way the YAMS Live Tuner does. One robot runs every test: a second one would claim the same CAN
 * IDs.
 */
class ADRCMechanismsTest {
  private static final double kLoopSeconds = 0.02;
  private static final String kArm = "/Tuning/ADRCArm/ArmMotor/";
  private static final String kElevator = "/Tuning/ADRCElevator/ElevatorMotor/";
  private static final String kShooter = "/Tuning/ADRCShooter/ShooterMotor/";
  private static final String kArmCommand = "/Tuning/ADRCArmMechanism/Live Tuning/running";

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
    Scheduler.getDefault().cancelAll();
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
  void anADRCReplacesTheFeedbackGains() {
    for (String motor : new String[] {kArm, kElevator, kShooter}) {
      for (String gain : new String[] {"kP", "kI", "kD"}) {
        assertFalse(nt.getTopic(motor + "closedloop/feedback/" + gain).exists(), motor + " publishes no " + gain);
      }
      for (String field : new String[] {"b0", "settlingTime", "bandwidthScale"}) {
        assertTrue(nt.getTopic(motor + "closedloop/adrc/" + field).exists(), motor + " publishes adrc/" + field);
      }
    }
    assertEquals(0.3, nt.getEntry(kArm + "closedloop/adrc/settlingTime").getDouble(0), 1e-9);
    assertEquals(0.3, nt.getEntry(kElevator + "closedloop/adrc/settlingTime").getDouble(0), 1e-9);
    assertEquals(0.2, nt.getEntry(kShooter + "closedloop/adrc/settlingTime").getDouble(0), 1e-9);
  }

  @Test
  void theArmReachesItsAngle() {
    Scheduler.getDefault().schedule(robot.arm.setAngle(Degrees.of(45)));
    step(3);
    assertEquals(45, robot.arm.getAngle().in(Degrees), 2, "arm angle");
  }

  @Test
  void theElevatorReachesItsHeight() {
    Scheduler.getDefault().schedule(robot.elevator.setHeight(Meters.of(1)));
    step(3);
    assertEquals(1, robot.elevator.getHeight().in(Meters), 0.02, "elevator height");
  }

  @Test
  void theShooterReachesItsSpeed() {
    Scheduler.getDefault().schedule(robot.shooter.spinUp(RPM.of(3000)));
    step(2);
    assertEquals(3000, robot.shooter.getSpeed().in(RPM), 60, "shooter speed");
  }

  @Test
  void liveTuningRetunesTheArmsADRC() {
    set(kArm + "closedloop/setpoint/position", 30.0);
    set(kArmCommand, true);
    step(3);
    assertEquals(30, robot.arm.getAngle().in(Degrees), 2, "arm angle while live tuning");
    set(kArm + "closedloop/adrc/settlingTime", 0.8);
    step(0.1);
    assertEquals(0.8, robot.arm.getADRC().orElseThrow().getSettlingTime().in(Seconds), 1e-9,
        "the ADRC runs the new settling time");
    step(2);
    assertEquals(30, robot.arm.getAngle().in(Degrees), 2, "still holding after the retune");
    set(kArm + "closedloop/adrc/settlingTime", 0.3);
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
