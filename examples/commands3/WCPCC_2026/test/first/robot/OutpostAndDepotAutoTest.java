// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertTrue;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import first.robot.generated.ChoreoTraj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.SimHooks;

/**
 * Runs the "Outpost and Depot" opmode through the real OpModeRobot loop in simulation, the same way
 * the simulated driver station does: select the opmode while disabled, then enable autonomous.
 */
class OutpostAndDepotAutoTest {
    private static final double kLoopSeconds = 0.02;
    /** Allowed distance from an expected pose, in meters. */
    private static final double kTolerance = 0.5;
    /**
     * Allowed heading error while following the path, in degrees. Loose because the Phoenix
     * simulation runs on its own real-time thread, so an occasional run lags a little; a drivetrain
     * that loses control of its modules drifts 75 degrees or more.
     */
    private static final double kHeadingToleranceDegrees = 60;
    /** Loops within which a module turning around and back counts as a wiggle. */
    private static final int kFlipBackWindowLoops = 10;

    private Robot robot;
    private Thread robotThread;
    private Trajectory<SwerveSample> fullTrajectory;
    private int loop = 0;
    private final double[] lastModuleAngles = new double[4];
    private final int[] lastFlipLoop = {-100, -100, -100, -100};
    private int moduleFlipBacks = 0;

    @BeforeEach
    void startRobot() {
        assertTrue(HAL.initialize());
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        fullTrajectory = Choreo.<SwerveSample>loadTrajectory(ChoreoTraj.OutpostAndDepotTrajectory.name()).orElseThrow();

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
    void drivesTheRouteWithAndWithoutAnAlliance() {
        // Simulation starts with an unknown alliance; the routine must still run, as blue.
        selectOpModeAndEnable();
        step(0.1);
        assertNear(initialPose(false), "odometry reset to the blue start (unknown alliance)");
        // The path holds a 180 degree heading until the depot. A drivetrain that loses control of its
        // modules (e.g. wheels reversing back and forth) spins the robot off that heading.
        final double maxHeadingError = stepTrackingHeadingError(6.0, Rotation2d.k180deg);
        assertTrue(maxHeadingError < kHeadingToleranceDegrees,
            "heading held near 180 degrees to the depot, but drifted " + maxHeadingError + " degrees");
        // By 9 s the robot has driven start -> outpost -> depot -> shooting pose.
        step(2.9);
        assertNear(segmentEnd(2, false), "reached the blue shooting pose");
        // A module may turn around once when the path changes direction, but turning around and back
        // within a few loops is the wheel wiggling instead of rotating to where it needs to point. A
        // single one is tolerated: on a heavily loaded machine the Phoenix simulation, which runs in
        // real time, can lag the steer motors enough for a wheel to turn back once. A wheel that keeps
        // wiggling turns around and back many times.
        assertTrue(moduleFlipBacks <= 1, "modules turned around and back " + moduleFlipBacks + " times during the route");

        // Disabling ends the opmode; the next enable recreates it and runs mirrored for red.
        disable();
        DriverStationSim.setAllianceStationId(AllianceStationID.RED_1);
        DriverStationSim.notifyNewData();
        step(0.2);
        enable();
        step(0.1);
        assertNear(initialPose(true), "odometry reset to the red start");
    }

    private void selectOpModeAndEnable() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.AUTONOMOUS);
        DriverStationSim.setOpMode(OpModeOption.makeId(RobotMode.AUTONOMOUS, "Outpost and Depot".hashCode()));
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(0.2);
        enable();
    }

    private void enable() {
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
    }

    private void disable() {
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
    }

    private void step(double seconds) {
        for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
            stepOneLoop();
        }
    }

    /** Steps the robot and returns the largest heading error from {@code expected} seen. */
    private double stepTrackingHeadingError(double seconds, Rotation2d expected) {
        double maxError = 0;
        for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
            stepOneLoop();
            maxError = Math.max(maxError, Math.abs(pose().getRotation().minus(expected).getDegrees()));
        }
        return maxError;
    }

    /**
     * Advances one robot loop. Phoenix 6 simulates its devices on its own thread in real time, so
     * each simulated loop is also given real time, or the Talons would respond to commands a varying
     * number of loops late depending on machine load.
     */
    private void stepOneLoop() {
        SimHooks.stepTiming(kLoopSeconds);
        try {
            Thread.sleep((long) (kLoopSeconds * 1000));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        trackModuleFlips();
    }

    /** Counts modules whose commanded angle turns around (over 90 degrees) and back again quickly. */
    private void trackModuleFlips() {
        final SwerveModuleVelocity[] states = robot.swerve.getDesiredModuleStates();
        for (int m = 0; m < states.length; m++) {
            if (states[m] == null) {
                continue;
            }
            final double angle = states[m].angle.getDegrees();
            if (loop > 0 && Math.abs(Math.IEEEremainder(angle - lastModuleAngles[m], 360)) > 90) {
                if (loop - lastFlipLoop[m] <= kFlipBackWindowLoops) {
                    moduleFlipBacks++;
                }
                lastFlipLoop[m] = loop;
            }
            lastModuleAngles[m] = angle;
        }
        loop++;
    }

    private Pose2d initialPose(boolean red) {
        return fullTrajectory.getInitialPose(red).orElseThrow();
    }

    private Pose2d segmentEnd(int segment, boolean red) {
        return fullTrajectory.getSplit(segment).orElseThrow().getFinalPose(red).orElseThrow();
    }

    private Pose2d pose() {
        return robot.swerve.getPose();
    }

    private void assertNear(Pose2d expected, String message) {
        final Pose2d actual = pose();
        final double error = actual.getTranslation().getDistance(expected.getTranslation());
        assertTrue(error < kTolerance, message + ": expected " + expected + " but was " + actual);
    }
}
