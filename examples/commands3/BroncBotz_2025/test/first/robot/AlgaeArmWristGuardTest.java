// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.OperatorConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.hardware.hal.OpModeOption;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.simulation.NiDsXboxControllerSim;
import org.wpilib.simulation.SimHooks;

/**
 * The algae arm only swings down to rest once the coral wrist is at rest, or it would hit the wrist
 * and break. In the "Angular Velocity Teleop" opmode, the operator raises the algae arm to the reef, holds a
 * coral level that swings the wrist out, and lets go of the reef position, so the algae arm's default
 * command stows it: the algae arm waits above its stowed angle until the coral level is let go and
 * the wrist is back at rest.
 */
class AlgaeArmWristGuardTest {
    private static final double kLoopSeconds = 0.02;

    private Robot robot;
    private Thread robotThread;
    private NiDsXboxControllerSim operator;

    @BeforeEach
    void startRobot() {
        assertTrue(HAL.initialize());
        SimHooks.pauseTiming();
        DriverStationSim.resetData();
        robot = new Robot();
        robotThread = new Thread(robot::startCompetition);
        robotThread.start();
        SimHooks.waitForProgramStart();
        operator = new NiDsXboxControllerSim(OperatorConstants.kOperatorControllerPort);
        operator.setButtonsAvailable(-1);
        operator.setButtonsMaximumIndex(32);
        operator.setPOVsAvailable(1);
        operator.setPOVsMaximumIndex(1);
    }

    @AfterEach
    void stopRobot() throws InterruptedException {
        robot.endCompetition();
        robotThread.join();
        robot.close();
        SimHooks.resumeTiming();
    }

    @Test
    void algaeArmWaitsForTheWristBeforeResting() {
        DriverStationSim.setDsAttached(true);
        DriverStationSim.setRobotMode(RobotMode.TELEOPERATED);
        DriverStationSim.setOpMode(OpModeOption.makeId(RobotMode.TELEOPERATED, "Angular Velocity Teleop".hashCode()));
        DriverStationSim.setEnabled(false);
        DriverStationSim.notifyNewData();
        step(0.2);
        DriverStationSim.setEnabled(true);
        DriverStationSim.notifyNewData();
        step(0.5);

        // Raise the algae arm to the low reef algae.
        operator.setPOV(POVDirection.DOWN);
        operator.notifyNewData();
        step(3);
        assertTrue(robot.algaeArm.getAngle().gt(AlgaeArmConstants.kStowed.plus(Degrees.of(20))),
            "raised the algae arm above its stowed angle, but it was at " + robot.algaeArm.getAngle().in(Degrees));

        // Swing the coral wrist out for L2 while the algae arm is at the reef, then let go of the reef
        // position while the wrist is still out: the algae arm's default command stows it.
        operator.setBButton(true);
        operator.notifyNewData();
        step(1);
        assertFalse(robot.coralWrist.isAtRest(), "swung the coral wrist out");
        operator.setPOV(POVDirection.CENTER);
        operator.notifyNewData();
        step(2);
        assertTrue(robot.algaeArm.getAngle().gt(AlgaeArmConstants.kStowed.plus(AlgaeArmConstants.kTolerance)),
            "the algae arm waited above its stowed angle for the coral wrist, but was at " + robot.algaeArm.getAngle().in(Degrees));

        // Let go of the coral level: the wrist rests, and only then does the algae arm stow.
        operator.setBButton(false);
        operator.notifyNewData();
        step(3);
        assertTrue(robot.coralWrist.isAtRest(), "the coral wrist went back to rest");
        assertTrue(robot.algaeArm.isNear(AlgaeArmConstants.kStowed),
            "the algae arm stowed once the coral wrist was at rest, but was at " + robot.algaeArm.getAngle().in(Degrees));
    }

    private void step(double seconds) {
        for (int i = 0; i < Math.round(seconds / kLoopSeconds); i++) {
            SimHooks.stepTiming(kLoopSeconds);
        }
    }
}
