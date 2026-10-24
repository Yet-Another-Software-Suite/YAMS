// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RPM;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Live tunes the double flywheel over NetworkTables the way the YAMS Live Tuner does: start the mechanism's one "Live
 * Tuning" command, then write each wheel's values under its own {@code /Tuning/<wheel>} table.
 *
 * <p>One robot runs every test: a second one would claim the same CAN IDs. Each test starts with Live Tuning stopped
 * and both wheels back at their starting gains.
 */
class DoubleFlyWheelLiveTuningTest {
  private static final double kLoopSeconds = 0.02;
  private static final String kCommand = "/Tuning/DoubleFlyWheelMechanism/Live Tuning/running";
  private static final String kLower = "/Tuning/LowerFlyWheel/LowerMotor/";
  private static final String kUpper = "/Tuning/UpperFlyWheel/UpperMotor/";

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
    set(kCommand, false);
    step(0.1);
    for (String wheel : new String[] {kLower, kUpper}) {
      set(wheel + "voltage/enabled", false);
      set(wheel + "voltage/setpoint", 0.0);
      set(wheel + "closedloop/feedforward/kV", 0.1);
    }
  }

  @AfterAll
  static void stopRobot() throws InterruptedException {
    set(kCommand, false);
    robot.endCompetition();
    robotThread.join();
    robot.close();
    SimHooks.resumeTiming();
  }

  @Test
  void bothWheelsShareOneLiveTuningCommand() {
    assertTrue(nt.getTopic(kCommand).exists(), "the mechanism publishes one Live Tuning command");
    assertFalse(nt.getTopic("/Tuning/LowerFlyWheel/Live Tuning/running").exists());
    assertFalse(nt.getTopic("/Tuning/UpperFlyWheel/Live Tuning/running").exists());
  }

  @Test
  void eachWheelFollowsItsOwnSetpoint() {
    set(kLower + "closedloop/setpoint/velocity", 3000.0);
    set(kUpper + "closedloop/setpoint/velocity", 1500.0);
    set(kCommand, true);
    step(3);
    assertTrue(nt.getEntry(kCommand).getBoolean(false), "Live Tuning is running");
    // The feedforward starts low, so both settle a little short of their setpoints.
    final double lower = robot.doubleFlyWheel.getLowerSpeed().in(RPM);
    final double upper = robot.doubleFlyWheel.getUpperSpeed().in(RPM);
    assertEquals(3000, lower, 750, "lower wheel speed");
    assertEquals(1500, upper, 375, "upper wheel speed");
    assertTrue(lower > upper + 750, "the wheels run at their own speeds: lower " + lower + ", upper " + upper);
  }

  @Test
  void eachWheelHasItsOwnGains() {
    set(kLower + "closedloop/setpoint/velocity", 2000.0);
    set(kUpper + "closedloop/setpoint/velocity", 2000.0);
    set(kCommand, true);
    step(3);
    final double before = robot.doubleFlyWheel.getUpperSpeed().in(RPM);
    // Only the upper wheel's kV goes up, to about what holds a NEO at speed.
    set(kUpper + "closedloop/feedforward/kV", 0.127);
    step(3);
    final double lower = robot.doubleFlyWheel.getLowerSpeed().in(RPM);
    final double upper = robot.doubleFlyWheel.getUpperSpeed().in(RPM);
    assertTrue(upper > before + 100, "the upper wheel gets closer with more kV: " + before + " to " + upper);
    assertEquals(2000, upper, 100, "upper wheel speed with kV tuned");
    assertTrue(upper > lower + 100, "the lower wheel's gains did not change: lower " + lower + ", upper " + upper);
  }

  @Test
  void voltageDrivesOnlyItsWheel() {
    set(kLower + "closedloop/setpoint/velocity", 1000.0);
    set(kUpper + "closedloop/setpoint/velocity", 1000.0);
    set(kCommand, true);
    step(2);
    final double lowerBefore = robot.doubleFlyWheel.getLowerSpeed().in(RPM);
    final double upperBefore = robot.doubleFlyWheel.getUpperSpeed().in(RPM);
    // Turning voltage on starts it at 0 V, so the voltage is sent once it is on, as the tuner does.
    set(kUpper + "voltage/enabled", true);
    step(0.1);
    set(kUpper + "voltage/setpoint", 6.0);
    step(2);
    final double lower = robot.doubleFlyWheel.getLowerSpeed().in(RPM);
    final double upper = robot.doubleFlyWheel.getUpperSpeed().in(RPM);
    assertTrue(upper > upperBefore + 1000, "6 V spins the upper wheel up: " + upperBefore + " to " + upper);
    assertEquals(lowerBefore, lower, 50, "the lower wheel holds its setpoint");
  }

  @Test
  void stoppingLiveTuningTurnsOffEveryWheelsVoltage() {
    set(kCommand, true);
    step(0.2);
    set(kLower + "voltage/enabled", true);
    set(kUpper + "voltage/enabled", true);
    step(0.1);
    set(kLower + "voltage/setpoint", 3.0);
    set(kUpper + "voltage/setpoint", 3.0);
    step(1);
    assertTrue(Math.abs(robot.doubleFlyWheel.getLowerSpeed().in(RPM)) > 500, "the lower wheel spins up");
    set(kCommand, false);
    step(0.2);
    assertFalse(nt.getEntry(kLower + "voltage/enabled").getBoolean(true), "the lower wheel's voltage is off");
    assertFalse(nt.getEntry(kUpper + "voltage/enabled").getBoolean(true), "the upper wheel's voltage is off");
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
