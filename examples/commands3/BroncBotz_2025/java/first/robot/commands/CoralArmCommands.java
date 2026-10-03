// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.CoralArmConstants;
import first.robot.mechanisms.CoralArm;
import first.robot.mechanisms.CoralRoller;
import first.robot.mechanisms.CoralWrist;
import first.robot.mechanisms.Elevator;
import first.robot.mechanisms.Swerve;
import first.robot.util.ReefTargeting;
import org.wpilib.command3.Command;
import org.wpilib.command3.Coroutine.WaitResult;
import org.wpilib.units.measure.Angle;

/**
 * Coral arm commands that use several mechanisms: scoring coral on the reef, loading it at the human
 * player station, and moving to the operator's coral presets.
 */
public class CoralArmCommands {
    private final Swerve swerve;
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final CoralWrist coralWrist;
    private final CoralRoller coralRoller;
    private final ReefTargeting targeting;

    public CoralArmCommands(Swerve swerve, Elevator elevator, CoralArm coralArm, CoralWrist coralWrist, CoralRoller coralRoller,
                            ReefTargeting targeting) {
        this.swerve = swerve;
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.coralWrist = coralWrist;
        this.coralRoller = coralRoller;
        this.targeting = targeting;
    }

    private Angle targetCoralAngle() {
        return CoralArm.coralAngle(targeting.getLevel());
    }

    /**
     * Score a coral on the targeted branch. The algae arm stays stowed out of the way throughout, as
     * its default command keeps it, and once done the coral arm and elevator go back to rest.
     */
    public Command scoreCoral() {
        return Command.noRequirements(coroutine -> {
            final ReefTargeting.Level level = targeting.getLevel();
            final Angle armAngle = CoralArm.coralAngle(level);
            // Pre-score, with the robot stopped: the elevator rises to the branch's scoring height while
            // the coral arm swings to its angle for the level and the wrist to its scoring angle. The
            // elevator and the wrist hold there until the coral is scored.
            coroutine.fork(swerve.stopCommand(), elevator.holdCoralLevel(() -> level), coralArm.holdAt(armAngle), coralWrist.swingOut(), coralRoller.score());
            // Only drive to the reef once all three are there, giving up if they are not within two
            // seconds.
            final WaitResult ready = coroutine.waitUntil(
                () -> elevator.isAtCoralLevel(level) && coralArm.isNear(armAngle) && coralWrist.isAtScoringAngle(), Seconds.of(2));
            if (ready == WaitResult.TIMED_OUT) {
                return;
            }
            coroutine.await(swerve.driveToPose(targeting::getCoralScoringPose));
            // Swing the coral arm down onto the branch until the coral has left the intake, taking the
            // arm over from holding its angle.
            coroutine.await(coralArm.score().withTimeout(Seconds.of(1)));
            // Back off from the reef with the arm still down.
            coroutine.fork(coralArm.holdCurrent());
            coroutine.await(swerve.backOff().withTimeout(Seconds.of(0.5)));
        }).named("Score Coral");
    }

    /**
     * Move the coral arm, then the elevator, to a level, with the wrist at an angle, holding them
     * there until canceled. Holding them keeps their default commands from taking them back to rest.
     */
    public Command coralLevel(ReefTargeting.Level level, Angle wristAngle) {
        return Command.noRequirements(coroutine -> {
            final Angle armAngle = CoralArm.coralAngle(level);
            coroutine.fork(coralWrist.holdAt(wristAngle), coralArm.holdAt(armAngle));
            coroutine.waitUntil(() -> coralArm.isNear(armAngle));
            coroutine.fork(elevator.holdCoralLevel(() -> level));
            // Hold them there until the button is released, like the original.
            coroutine.park();
        }).named("Coral " + level);
    }

    /** Hold the coral arm at the human player station angle while intaking. */
    public Command intakeFromHumanPlayer() {
        final Command arm = coralArm.holdAt(CoralArmConstants.HP);
        final Command intake = coralRoller.intake();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, intake)).named("Intake Human Player");
    }

    /** Swing the wrist out and hold the coral in, as when scoring, until canceled. */
    public Command holdToScore() {
        final Command wrist = coralWrist.swingOut();
        final Command roller = coralRoller.score();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(wrist, roller)).named("Hold Coral to Score");
    }

    /** Swing the wrist out and push the coral out, until canceled. */
    public Command outtake() {
        final Command wrist = coralWrist.swingOut();
        final Command roller = coralRoller.outtake();
        return Command.noRequirements(coroutine -> coroutine.awaitAll(wrist, roller)).named("Outtake Coral");
    }

    /** Hold the coral arm and elevator at a level, until canceled. */
    public Command holdCoralLevel(ReefTargeting.Level level) {
        final Command arm = coralArm.holdAt(CoralArm.coralAngle(level));
        final Command lift = elevator.holdCoralLevel(() -> level);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(arm, lift)).named("Hold Coral " + level);
    }
}
