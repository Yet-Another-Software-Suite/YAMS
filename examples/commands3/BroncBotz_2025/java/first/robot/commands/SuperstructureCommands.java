// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.mechanisms.AlgaeArm;
import first.robot.mechanisms.CoralArm;
import first.robot.mechanisms.Elevator;
import org.wpilib.command3.Command;
import org.wpilib.units.measure.Time;

/**
 * Commands that move both arms, such as stowing them, and a helper the coral and algae arm commands
 * share. This was the original's {@code ScoringSystem} and {@code LoadingSystem}, now split into
 * these, {@link CoralArmCommands} and {@link AlgaeArmCommands}; only the commands that were bound or
 * used by the autonomous routine were kept.
 */
public class SuperstructureCommands {
    private final Elevator elevator;
    private final CoralArm coralArm;
    private final AlgaeArm algaeArm;

    public SuperstructureCommands(Elevator elevator, CoralArm coralArm, AlgaeArm algaeArm) {
        this.elevator = elevator;
        this.coralArm = coralArm;
        this.algaeArm = algaeArm;
    }

    /** Start a command after a delay. */
    static Command after(Time delay, Command command) {
        return Command.noRequirements(coroutine -> {
            coroutine.wait(delay);
            coroutine.await(command);
        }).named(command.name() + " after " + delay);
    }

    /** Move both arms to their stowed angle. */
    public Command stowArms() {
        final Command algae = algaeArm.moveTo(AlgaeArmConstants.kStowed);
        final Command coral = coralArm.moveTo(CoralArmConstants.kStowed);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(algae, coral)).named("Stow Arms");
    }

    /** Both arms to their fully stowed angle, until canceled. */
    public Command fullyStowArms() {
        final Command algae = algaeArm.holdAt(AlgaeArmConstants.kFullyStowed);
        final Command coral = coralArm.holdAt(CoralArmConstants.kFullyStowed);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(algae, coral)).named("Fully Stow Arms");
    }

    /** Raise the elevator clear, then hold an arm at an angle. */
    public Command clearElevatorThen(Command arm) {
        final Command lift = elevator.holdAt(ElevatorConstants.kClearHeight);
        final Command delayedArm = after(Seconds.of(0.3), arm);
        return Command.noRequirements(coroutine -> coroutine.awaitAll(lift, delayedArm)).named("Clear Elevator Then " + arm.name());
    }
}
