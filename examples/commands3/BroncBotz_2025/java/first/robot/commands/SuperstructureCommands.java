// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.mechanisms.AlgaeArm;
import first.robot.mechanisms.AlgaeIntake;
import first.robot.mechanisms.CoralArm;
import first.robot.mechanisms.CoralIntake;
import first.robot.mechanisms.Elevator;
import first.robot.mechanisms.Swerve;
import first.robot.util.ReefTargeting;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Time;

/**
 * Commands that use several mechanisms: scoring coral, pulling and scoring algae, and moving to the
 * operator's preset positions. This was the original's {@code ScoringSystem} and
 * {@code LoadingSystem}; only the commands that were bound or used by the autonomous routine were
 * kept.
 *
 * <p>Each command is a coroutine without requirements of its own. The original's parallel groups
 * with {@code until}, {@code withTimeout} and {@code withDeadline} become {@link #phase} and
 * {@link #deadline} steps: each forks the mechanism commands and finishes when its condition, timeout
 * or deadline command does, which cancels the commands it forked, so the next step starts clean.
 */
public class SuperstructureCommands {
    private final Swerve swerve;
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final CoralIntake coralIntake;
    private final AlgaeArm algaeArm;
    private final AlgaeIntake algaeIntake;
    private final ReefTargeting targeting;

    public SuperstructureCommands(Swerve swerve, Elevator elevator, CoralArm coralArm, CoralIntake coralIntake,
                                  AlgaeArm algaeArm, AlgaeIntake algaeIntake, ReefTargeting targeting) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.coralIntake = coralIntake;
        this.algaeArm = algaeArm;
        this.algaeIntake = algaeIntake;
        this.targeting = targeting;
    }

    /** Run commands together until a condition is true or a timeout passes. */
    private static Command phase(String name, BooleanSupplier until, Time timeout, Command... commands) {
        return Command.noRequirements(coroutine -> {
            coroutine.fork(commands);
            coroutine.waitUntil(until, timeout);
        }).named(name);
    }

    /** Run commands alongside a deadline command, until the deadline finishes. */
    private static Command deadline(String name, Command deadline, Command... commands) {
        return Command.noRequirements(coroutine -> {
            coroutine.fork(commands);
            coroutine.await(deadline);
        }).named(name);
    }

    /** Start a command after a delay. */
    private static Command after(Time delay, Command command) {
        return Command.noRequirements(coroutine -> {
            coroutine.wait(delay);
            coroutine.await(command);
        }).named(command.name() + " after " + delay);
    }

    private Angle targetCoralAngle() {
        return CoralArm.coralAngle(targeting.getLevel());
    }

    /**
     * Raise to the targeted level, drive to the targeted branch, swing the coral onto it and back
     * away.
     */
    public Command scoreCoral() {
        // Raise first, for at most two seconds.
        final Command raise = phase("Raise",
            () -> elevator.isAtCoralLevel(targeting.getLevel()) && coralArm.isNear(targetCoralAngle())
                && coralIntake.isAtScoringAngle(),
            Seconds.of(2),
            swerve.stopCommand(),
            elevator.holdCoralLevel(targeting::getLevel),
            algaeArm.holdAt(AlgaeArmConstants.kStowed),
            Command.noRequirements(coroutine -> {
                coroutine.await(coralArm.moveTo(this::targetCoralAngle));
                coroutine.await(coralArm.holdCurrent());
            }).named("CoralArm to Level"),
            after(Seconds.of(0.3), coralIntake.score()));
        // Then drive in while holding.
        final Command driveIn = deadline("Drive In",
            swerve.driveToPose(targeting::getCoralScoringPose),
            elevator.holdCoralLevel(targeting::getLevel),
            coralArm.holdCurrent(),
            algaeArm.holdAt(AlgaeArmConstants.kStowed),
            after(Seconds.of(0.3), coralIntake.score()));
        final Command place = deadline("Place",
            coralArm.score().withTimeout(Seconds.of(1)),
            coralIntake.score());
        final Command backOff = phase("Back Off", () -> false, Seconds.of(0.5),
            swerve.backOff(), coralIntake.intake(), coralArm.holdCurrent());
        return Command.noRequirements(coroutine -> {
            coroutine.await(raise);
            coroutine.await(driveIn);
            coroutine.await(place);
            coroutine.await(backOff);
        }).named("Score Coral");
    }

    /**
     * Reach the targeted reef face's algae, drive in with the intake running, lift it off and back
     * away.
     */
    public Command loadAlgae() {
        final Command reach = phase("Reach",
            () -> algaeArm.isNear(AlgaeArm.algaeAngle(targeting.getLevel()))
                && elevator.isNear(Elevator.algaeHeight(targeting.getLevel())),
            Seconds.of(2.5),
            swerve.stopCommand(),
            elevator.holdAlgaeLevel(targeting::getLevel),
            coralArm.holdAt(CoralArmConstants.kAlgaeClearance),
            algaeArm.holdAlgaeLevel(targeting::getLevel));
        final Command driveIn = deadline("Drive In",
            swerve.driveToPose(targeting::getAlgaeScoringPose),
            elevator.holdAlgaeLevel(targeting::getLevel),
            algaeArm.holdAlgaeLevel(targeting::getLevel),
            algaeIntake.intake(),
            coralArm.holdAt(CoralArmConstants.kAlgaeClearance));
        // The lift never finishes on its own, so this ends once the algae is in or after a second.
        final Command lift = phase("Lift", algaeArm::isAlgaeLoaded, Seconds.of(1),
            algaeArm.lift(),
            algaeIntake.intake(),
            elevator.holdAlgaeLevel(targeting::getLevel),
            swerve.lockPose(),
            coralArm.holdAt(CoralArmConstants.kAlgaeClearance));
        final Command backOff = phase("Back Off", () -> false, Seconds.of(1),
            swerve.backOff(),
            elevator.holdAlgaeLevel(targeting::getLevel),
            algaeIntake.intake(),
            coralArm.holdAt(CoralArmConstants.kAlgaeClearance));
        return Command.noRequirements(coroutine -> {
            coroutine.await(reach);
            coroutine.await(driveIn);
            coroutine.await(lift);
            coroutine.await(backOff);
        }).named("Load Algae");
    }

    /** Raise the algae to the net and throw it. */
    public Command scoreAlgaeNet() {
        // The original checked the processor angle here, so it always waited the full 5 seconds.
        final Command raise = phase("Raise",
            () -> elevator.isNear(ElevatorConstants.Algae.NET) && algaeArm.isNear(AlgaeArmConstants.NET),
            Seconds.of(5),
            algaeArm.holdAt(AlgaeArmConstants.NET),
            elevator.holdAt(ElevatorConstants.Algae.NET));
        final Command shoot = phase("Shoot", algaeArm::isAlgaeScored, Seconds.of(2),
            algaeArm.holdAt(AlgaeArmConstants.NET),
            elevator.holdAt(ElevatorConstants.Algae.NET),
            algaeIntake.outtake());
        return Command.noRequirements(coroutine -> {
            coroutine.await(raise);
            coroutine.await(shoot);
        }).named("Score Algae Net");
    }

    /** Swing both arms to their stowed angle with the wrist at rest, until canceled. */
    public Command restArmsSafe() {
        final Command coral = coralArm.holdAt(CoralArmConstants.kStowed);
        final Command algae = algaeArm.holdAt(AlgaeArmConstants.kStowed);
        final Command wrist = coralIntake.rest();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(coral, algae, wrist)).named("Rest Arms Safe");
    }

    /** Move the coral arm, then the elevator, to a level, holding the wrist at an angle. */
    public Command coralLevel(ReefTargeting.Level level, Angle wristAngle) {
        final Command arm = coralArm.moveTo(CoralArm.coralAngle(level));
        final Command lift = elevator.moveToCoralLevel(level);
        final Command wrist = coralIntake.holdWrist(wristAngle);
        return Command.noRequirements(coroutine -> {
            coroutine.fork(wrist);
            coroutine.await(arm);
            coroutine.await(lift);
            // Keep holding the wrist until the button is released, like the original.
            coroutine.park();
        }).named("Coral " + level);
    }

    /** Hold the coral arm at the human player station angle while intaking. */
    public Command intakeFromHumanPlayer() {
        final Command arm = coralArm.holdAt(CoralArmConstants.HP);
        final Command intake = coralIntake.intake();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, intake)).named("Intake Human Player");
    }

    /** Hold the algae arm and elevator at a reef algae position. */
    public Command algaeReefPosition(ReefTargeting.Level level) {
        final Command arm = algaeArm.holdAt(AlgaeArm.algaeAngle(level));
        final Command lift = elevator.holdAt(Elevator.algaeHeight(level));
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, lift)).named("Algae Reef " + level);
    }

    /** Move both arms to their stowed angle. */
    public Command stowArms() {
        final Command algae = algaeArm.moveTo(AlgaeArmConstants.kStowed);
        final Command coral = coralArm.moveTo(CoralArmConstants.kStowed);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(algae, coral)).named("Stow Arms");
    }

    /** Raise the elevator clear, then hold an arm at an angle. */
    public Command clearElevatorThen(Command arm) {
        final Command lift = elevator.holdAt(ElevatorConstants.kClearHeight);
        final Command delayedArm = after(Seconds.of(0.3), arm);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(lift, delayedArm)).named("Clear Elevator Then " + arm.name());
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

    /** Both arms to their fully stowed angle, until canceled. */
    public Command fullyStowArms() {
        final Command algae = algaeArm.holdAt(AlgaeArmConstants.kFullyStowed);
        final Command coral = coralArm.holdAt(CoralArmConstants.kFullyStowed);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(algae, coral)).named("Fully Stow Arms");
    }

    /** Hold the coral arm and elevator at a level, until canceled. */
    public Command holdCoralLevel(ReefTargeting.Level level) {
        final Command arm = coralArm.holdAt(CoralArm.coralAngle(level));
        final Command lift = elevator.holdCoralLevel(() -> level);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, lift)).named("Hold Coral " + level);
    }
}
