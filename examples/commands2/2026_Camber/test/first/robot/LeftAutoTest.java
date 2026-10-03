// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the "Left Auto" PathPlanner auto through the real robot loop in simulation: the odometry
 * reset to the first path's start, following "Left Shot", then holding still while shooting.
 */
class LeftAutoTest {
    private static final double kLoopSeconds = 0.02;
    /** Allowed distance from an expected pose, in meters. */
    private static final double kTolerance = 0.3;
    /** Allowed heading error, in degrees. */
    private static final double kHeadingToleranceDegrees = 10;

    // Start and end of the "Left Shot" path, from deploy/pathplanner/paths/Left Shot.path.
    private static final Pose2d kLeftShotStart = new Pose2d(3.52151212553495, 7.379643366619116, Rotation2d.ZERO);
    private static final Pose2d kLeftShotEnd =
        new Pose2d(3.029843081312411, 4.778972895863053, Rotation2d.fromDegrees(-30.529705899934104));

    private Robot robot;
    private Thread robotThread;

    @BeforeEach
    void startRobot() {
        assertTrue(HAL.initialize());
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
        DriverStationSim.notifyNewData();

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
    void followsTheFirstPathThenShoots() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(0.2);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();

        step(0.1);
        assertNear(kLeftShotStart, "odometry reset to the start of Left Shot");
        // Left Shot takes under 2.5 s; the robot then shoots in place for 4 s.
        step(3.0);
        assertNear(kLeftShotEnd, "reached the end of Left Shot");
        step(1.0);
        assertNear(kLeftShotEnd, "held the shooting pose");
    }

    private void step(double seconds) {
        for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
            SimHooks.stepTiming(kLoopSeconds);
            // Phoenix 6 simulates its devices on its own real-time thread.
            try {
                Thread.sleep((long) (kLoopSeconds * 1000));
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private Pose2d pose() {
        return robot.getRobotContainer().getDrivebase().getPose();
    }

    private void assertNear(Pose2d expected, String message) {
        final Pose2d actual = pose();
        final double error = actual.getTranslation().getDistance(expected.getTranslation());
        final double headingError = Math.abs(actual.getRotation().minus(expected.getRotation()).getDegrees());
        assertTrue(error < kTolerance && headingError < kHeadingToleranceDegrees,
            message + ": expected " + expected + " but was " + actual);
    }
}
