// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import static first.robot.commands.SuperstructureCommands.after;
import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.mechanisms.AlgaeArm;
import first.robot.mechanisms.AlgaeIntake;
import first.robot.mechanisms.CoralArm;
import first.robot.mechanisms.Elevator;
import first.robot.mechanisms.Swerve;
import first.robot.util.ReefTargeting;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.Angle;

/**
 * Algae arm commands that use several mechanisms: loading algae off the reef, scoring it in the net
 * or the processor, and moving to the operator's algae presets.
 */
public class AlgaeArmCommands {
    private final Swerve swerve;
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final AlgaeArm algaeArm;
    private final AlgaeIntake algaeIntake;
    private final ReefTargeting targeting;

    public AlgaeArmCommands(Swerve swerve, Elevator elevator, CoralArm coralArm, AlgaeArm algaeArm, AlgaeIntake algaeIntake, ReefTargeting targeting) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.algaeArm = algaeArm;
        this.algaeIntake = algaeIntake;
        this.targeting = targeting;
    }

    /**
     * Reach the targeted reef face's algae, drive in with the intake running, lift it off and back
     * away.
     */
    public Command loadAlgae() {
        return Command.noRequirements(coroutine -> {
            final ReefTargeting.Level level = targeting.getLevel();
            final Angle algaeAngle = AlgaeArm.algaeAngle(level);
            // Throughout, the elevator holds the algae height and the coral arm stays clear of it.
            coroutine.fork(elevator.holdAlgaeLevel(() -> level), coralArm.holdAt(CoralArmConstants.kAlgaeClearance));
            // Reach the algae with the robot stopped, for at most 2.5 seconds.
            coroutine.fork(swerve.stopCommand(), algaeArm.holdAt(algaeAngle));
            coroutine.waitUntil(() -> algaeArm.isNear(algaeAngle) && elevator.isNear(Elevator.algaeHeight(level)), Seconds.of(2.5));
            // Drive in with the intake running, until the routine ends.
            coroutine.fork(algaeIntake.intake());
            coroutine.await(swerve.driveToPose(targeting::getAlgaeScoringPose));
            // Lift the algae off with the wheels locked, until it is in or for at most a second.
            coroutine.fork(algaeArm.lift(), swerve.lockPose());
            coroutine.waitUntil(algaeArm::isAlgaeLoaded, Seconds.of(1));
            coroutine.await(swerve.backOff().withTimeout(Seconds.of(1)));
        }).named("Load Algae");
    }

    /** Raise the algae to the net and throw it. */
    public Command scoreAlgaeNet() {
        return Command.noRequirements(coroutine -> {
            coroutine.fork(algaeArm.holdAt(AlgaeArmConstants.NET), elevator.holdAt(ElevatorConstants.Algae.NET));
            // The original checked the processor angle here, so it always waited the full 5 seconds.
            coroutine.waitUntil(() -> elevator.isNear(ElevatorConstants.Algae.NET) && algaeArm.isNear(AlgaeArmConstants.NET), Seconds.of(5));
            // Throw it, until it is out or for at most two seconds.
            coroutine.await(algaeIntake.outtakeUntil(algaeArm::isAlgaeScored).withTimeout(Seconds.of(2)));
        }).named("Score Algae Net");
    }

    /** Hold the algae arm and elevator at a reef algae position. */
    public Command algaeReefPosition(ReefTargeting.Level level) {
        final Command arm = algaeArm.holdAt(AlgaeArm.algaeAngle(level));
        final Command lift = elevator.holdAt(Elevator.algaeHeight(level));
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, lift)).named("Algae Reef " + level);
    }

    /** The algae arm to the processor while outtaking, for the operator's D-pad. */
    public Command processor() {
        final Command arm = algaeArm.moveTo(AlgaeArmConstants.PROCESSOR);
        final Command outtake = algaeIntake.outtake();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, outtake)).named("Processor");
    }

    /** Hold the algae arm at the processor and outtake after 0.3 s, for the Launchpad. */
    public Command processorDelayed() {
        final Command arm = algaeArm.holdAt(AlgaeArmConstants.PROCESSOR);
        final Command outtake = after(Seconds.of(0.3), algaeIntake.outtake());
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, outtake)).named("Processor Delayed");
    }
}
