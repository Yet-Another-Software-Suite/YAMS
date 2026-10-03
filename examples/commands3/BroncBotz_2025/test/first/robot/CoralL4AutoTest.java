// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import first.robot.util.ReefTargeting;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the "Coral L4 H" opmode through the real OpModeRobot loop in simulation: the robot drives to
 * branch H, scores its preloaded coral and backs away.
 */
class CoralL4AutoTest {
    private static final double kLoopSeconds = 0.02;
    /** Allowed distance from the scoring pose after backing away, in meters. */
    private static final double kTolerance = 0.75;

    private Robot robot;
    private Thread robotThread;

    @BeforeEach
    void startRobot() {
        assertTrue(HAL.initialize());
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        robot = new Robot();
        robotThread = new Thread(robot::startCompetition);
        robotThread.start();
        SimHooks.waitForProgramStart();
    }

    @AfterEach
    void stopRobot() throws InterruptedException {
        robot.endCompetition();
        robotThread.join();
        robot.close();
        SimHooks.resumeTiming();
    }

    @Test
    void scoresOnBranchH() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
        DriverStationSim.setOpMode(OpModeOption.makeId(RobotMode.AUTONOMOUS, "Coral L4 H".hashCode()));
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(0.2);
        assertTrue(robot.coralArm.isCoralLoaded(), "starts with a preloaded coral");

        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(15);

        final ReefTargeting expected = new ReefTargeting();
        expected.setTarget(ReefTargeting.Branch.H, ReefTargeting.Level.L4);
        final Pose2d scoringPose = expected.getCoralScoringPose().orElseThrow();
        final Pose2d pose = robot.swerve.getPose();
        assertTrue(pose.getTranslation().getDistance(scoringPose.getTranslation()) < kTolerance,
            "drove to branch H at " + scoringPose + " but ended at " + pose);
        assertFalse(robot.coralArm.isCoralLoaded(), "scored the coral");
    }

    private void step(double seconds) {
        for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
            SimHooks.stepTiming(kLoopSeconds);
        }
    }
}
