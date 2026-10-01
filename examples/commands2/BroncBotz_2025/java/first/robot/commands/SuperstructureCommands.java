// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.commands;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.subsystems.AlgaeArm;
import first.robot.subsystems.AlgaeIntake;
import first.robot.subsystems.CoralArm;
import first.robot.subsystems.CoralIntake;
import first.robot.subsystems.Elevator;
import first.robot.subsystems.Swerve;
import first.robot.util.ReefTargeting;
import java.util.Set;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.units.measure.Angle;

/**
 * Commands that use several subsystems: scoring coral, pulling and scoring algae, and moving to the
 * operator's preset positions. This was the original's {@code ScoringSystem} and
 * {@code LoadingSystem}; only the commands that were bound or used by the autonomous routine were
 * kept.
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

    /** Drive to the targeted branch, or do nothing without a target. */
    private Command driveToCoralTarget() {
        return Commands.defer(() -> targeting.getCoralScoringPose()
            .map(pose -> swerve.driveToPose(() -> pose))
            .orElse(Commands.none()), Set.of(swerve));
    }

    /** Drive to the targeted reef face's algae, or do nothing without a target. */
    private Command driveToAlgaeTarget() {
        return Commands.defer(() -> targeting.getAlgaeScoringPose()
            .map(pose -> swerve.driveToPose(() -> pose))
            .orElse(Commands.none()), Set.of(swerve));
    }

    private Angle targetCoralAngle() {
        return CoralArm.coralAngle(targeting.getLevel());
    }

    private Command stowAlgaeArm() {
        return algaeArm.moveTo(AlgaeArmConstants.kStowed).andThen(algaeArm.holdCurrent());
    }

    /**
     * Raise to the targeted level, drive to the targeted branch, swing the coral onto it and back
     * away.
     */
    public Command scoreCoral() {
        return Commands.sequence(
            swerve.stopCommand(),
            // Raise first, for at most two seconds.
            Commands.parallel(
                elevator.holdCoralLevel(targeting::getLevel),
                stowAlgaeArm(),
                coralArm.moveToCoralLevel(targeting::getLevel).andThen(coralArm.holdCurrent()),
                Commands.waitSeconds(0.3).andThen(coralIntake.score()))
                .until(() -> elevator.isAtCoralLevel(targeting.getLevel()) && coralArm.isNear(targetCoralAngle())
                    && coralIntake.atScoringAngle.getAsBoolean())
                .withTimeout(2),
            // Then drive in while holding.
            Commands.parallel(
                elevator.holdCoralLevel(targeting::getLevel),
                coralArm.holdCurrent(),
                stowAlgaeArm(),
                Commands.waitSeconds(0.3).andThen(coralIntake.score()))
                .withDeadline(driveToCoralTarget()),
            coralIntake.score().withDeadline(coralArm.score().withTimeout(1)),
            swerve.backOff().alongWith(coralIntake.intake(), coralArm.holdCurrent()).withTimeout(0.5)
        ).withName("Score Coral");
    }

    /**
     * Reach the targeted reef face's algae, drive in with the intake running, lift it off and back
     * away.
     */
    public Command loadAlgae() {
        return Commands.sequence(
            swerve.stopCommand(),
            Commands.parallel(
                elevator.holdAlgaeLevel(targeting::getLevel),
                coralArm.holdAt(CoralArmConstants.kAlgaeClearance),
                algaeArm.moveToAlgaeLevel(targeting::getLevel).andThen(algaeArm.holdCurrent()))
                .until(() -> algaeArm.isNear(AlgaeArm.algaeAngle(targeting.getLevel()))
                    && elevator.isNear(Elevator.algaeHeight(targeting.getLevel())))
                .withTimeout(2.5),
            Commands.parallel(
                elevator.holdAlgaeLevel(targeting::getLevel),
                algaeArm.moveToAlgaeLevel(targeting::getLevel).andThen(algaeArm.holdCurrent()),
                algaeIntake.intake(),
                coralArm.holdAt(CoralArmConstants.kAlgaeClearance))
                .withDeadline(driveToAlgaeTarget()),
            Commands.parallel(
                algaeIntake.intake(),
                elevator.holdAlgaeLevel(targeting::getLevel),
                swerve.lockPose(),
                coralArm.holdAt(CoralArmConstants.kAlgaeClearance))
                .withDeadline(algaeArm.lift())
                .withTimeout(1)
                .until(algaeArm::isAlgaeLoaded),
            swerve.backOff()
                .alongWith(elevator.holdAlgaeLevel(targeting::getLevel), algaeIntake.intake(),
                    coralArm.holdAt(CoralArmConstants.kAlgaeClearance))
                .withTimeout(1)
        ).withName("Load Algae");
    }

    /** Raise the algae to the net and throw it. */
    public Command scoreAlgaeNet() {
        return Commands.parallel(
                algaeArm.moveTo(AlgaeArmConstants.NET).andThen(algaeArm.holdCurrent()),
                elevator.holdAt(ElevatorConstants.Algae.NET))
            // The original checked the processor angle here, so it always waited the full 5 seconds.
            .until(() -> elevator.isNear(ElevatorConstants.Algae.NET) && algaeArm.isNear(AlgaeArmConstants.NET))
            .withTimeout(5)
            .andThen(Commands.parallel(
                    algaeArm.moveTo(AlgaeArmConstants.NET).andThen(algaeArm.holdCurrent()),
                    elevator.holdAt(ElevatorConstants.Algae.NET),
                    algaeIntake.outtake())
                .withTimeout(2)
                .until(algaeArm::isAlgaeScored))
            .withName("Score Algae Net");
    }

    /** Swing both arms to their stowed angle with the wrist at rest, until interrupted. */
    public Command restArmsSafe() {
        return Commands.parallel(
            coralArm.holdAt(CoralArmConstants.kStowed),
            algaeArm.holdAt(AlgaeArmConstants.kStowed),
            coralIntake.rest()
        ).withName("Rest Arms Safe");
    }

    /** Move the coral arm, then the elevator, to a level, holding the wrist at an angle. */
    public Command coralLevel(ReefTargeting.Level level, Angle wristAngle) {
        return coralArm.moveTo(CoralArm.coralAngle(level))
            .andThen(elevator.moveToCoralLevel(level))
            .alongWith(coralIntake.holdWrist(wristAngle))
            .withName("Coral " + level);
    }

    /** Hold the coral arm at the human player station angle while intaking. */
    public Command intakeFromHumanPlayer() {
        return coralArm.holdAt(CoralArmConstants.HP).alongWith(coralIntake.intake()).withName("Intake Human Player");
    }

    /** Hold the algae arm and elevator at a reef algae position. */
    public Command algaeReefPosition(ReefTargeting.Level level) {
        return algaeArm.holdAt(AlgaeArm.algaeAngle(level)).alongWith(elevator.holdAt(Elevator.algaeHeight(level)));
    }

    /** Move both arms to their stowed angle. */
    public Command stowArms() {
        return algaeArm.moveTo(AlgaeArmConstants.kStowed).alongWith(coralArm.moveTo(CoralArmConstants.kStowed));
    }

    /** Raise the elevator clear, then hold an arm at an angle. */
    public Command clearElevatorThen(Command arm) {
        return elevator.holdAt(ElevatorConstants.kClearHeight).alongWith(Commands.waitSeconds(0.3).andThen(arm));
    }
}
