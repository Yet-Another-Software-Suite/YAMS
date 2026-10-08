// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.CoralIntakeConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.Robot;
import first.robot.util.Launchpad;
import first.robot.util.ReefTargeting.Level;
import first.robot.util.ReefTargeting.Side;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.util.Color;

/**
 * The original's driver, operator and Launchpad bindings, shared by every teleop opmode. Call from an opmode
 * constructor so they only exist while that opmode is selected.
 */
final class TeleopBindings {
    private TeleopBindings() {
        throw new UnsupportedOperationException("This is a utility class!");
    }

    /**
     * Bind the driver, operator and Launchpad controls. The driver's slow mode and alliance relative toggle adjust
     * the swerve drive's active input stream, so the opmode must set one first.
     *
     * @param robot The robot.
     */
    static void bindMechanisms(Robot robot) {
        configureDriverBindings(robot, robot.driver);
        configureOperatorBindings(robot, robot.operator);
        configureLaunchpadBindings(robot, robot.launchpad);
    }

    private static void configureDriverBindings(Robot robot, CommandNiDsXboxController driver) {
        // Slow translation while the left bumper is held.
        driver.leftBumper().whileTrue(Command.noRequirements(coroutine -> {
            robot.swerve.getInputStream().setTranslationAxisScale(OperatorConstants.kSlowTranslationScale);
            coroutine.park();
        }).whenCanceled(() -> robot.swerve.getInputStream().setTranslationAxisScale(OperatorConstants.kTranslationScale))
            .named("Slow Mode"));
        // X turns alliance relative control on (flipping the sticks for the red alliance) and Y turns it off. It
        // starts off, as in the original.
        driver.x().onTrue(Command.noRequirements(
            coroutine -> robot.swerve.getInputStream().setAllianceRelativeEnabled(true)).named("Alliance Relative On"));
        driver.y().onTrue(Command.noRequirements(
            coroutine -> robot.swerve.getInputStream().setAllianceRelativeEnabled(false)).named("Alliance Relative Off"));

        // Re-read the absolute encoders and swing the arm out.
        driver.b().whileTrue(Command.noRequirements(coroutine -> {
            robot.coralArm.synchronizeAbsoluteEncoder();
            coroutine.await(robot.coralArm.moveTo(CoralArmConstants.kStowed));
        }).named("Sync CoralArm"));
        driver.a().whileTrue(Command.noRequirements(coroutine -> {
            robot.algaeArm.synchronizeAbsoluteEncoder();
            coroutine.await(robot.algaeArm.moveTo(AlgaeArmConstants.kStowed));
        }).named("Sync AlgaeArm"));
    }

    private static void configureOperatorBindings(Robot robot, CommandNiDsXboxController operator) {
        operator.a().whileTrue(robot.coralCommands.coralLevel(Level.L1, CoralIntakeConstants.kRest));
        operator.b().whileTrue(robot.coralCommands.coralLevel(Level.L2, CoralIntakeConstants.kActive));
        operator.x().whileTrue(robot.coralCommands.coralLevel(Level.L3, CoralIntakeConstants.kActive));
        operator.y().whileTrue(robot.coralCommands.coralLevel(Level.L4, CoralIntakeConstants.kActive));
        operator.leftBumper().whileTrue(robot.coralCommands.outtake());
        operator.rightBumper().whileTrue(robot.algaeIntake.outtake());
        operator.rightTrigger().whileTrue(robot.algaeIntake.intake());
        operator.leftTrigger().whileTrue(robot.coralCommands.intakeFromHumanPlayer());
        // The D-pad triggers live on the generic HID in 2027.
        operator.getHID().povLeft().whileTrue(robot.algaeCommands.scoreAlgaeNet());
        operator.getHID().povRight().whileTrue(robot.algaeCommands.processor());
        operator.getHID().povDown().whileTrue(robot.algaeCommands.algaeReefPosition(Level.L2));
        operator.getHID().povUp().whileTrue(robot.algaeCommands.algaeReefPosition(Level.L3));
        operator.start().whileTrue(robot.superstructure.stowArms());
    }

    /** Pick a coral level, target the closest branch, and highlight the level. */
    private static Command selectCoralLevel(Robot robot, Level level) {
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            robot.targeting.setLevel(level);
            showSelectedLevel(robot, level);
        }).named("Select " + level);
    }

    /** Target the closest reef face's algae at a level, then pull it off. */
    private static Command loadAlgae(Robot robot, Level level) {
        final Command load = robot.algaeCommands.loadAlgae();
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            robot.targeting.setLevel(level);
            coroutine.await(load);
        }).named("Load Algae " + level);
    }

    /** Target the closest branch, then hold the coral arm and elevator at a level. */
    private static Command holdCoralLevel(Robot robot, Level level) {
        final Command hold = robot.coralCommands.holdCoralLevel(level);
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            coroutine.await(hold);
        }).named("Target and Hold " + level);
    }

    private static Command setSide(Robot robot, Side side) {
        return Command.noRequirements(coroutine -> robot.targeting.setSide(side)).named("Side " + side);
    }

    private static void configureLaunchpadBindings(Robot robot, Launchpad launchpad) {
        // Coral: pick the level (targets the closest branch), the side, then score.
        launchpad.bind(0, 1, Launchpad.kCoralLevel, selectCoralLevel(robot, Level.L4));
        launchpad.bind(0, 2, Launchpad.kCoralLevel, selectCoralLevel(robot, Level.L3));
        launchpad.bind(0, 3, Launchpad.kCoralLevel, selectCoralLevel(robot, Level.L2));
        launchpad.bind(0, 4, Launchpad.kCoralLevel, selectCoralLevel(robot, Level.L1));
        launchpad.bind(3, 0, Color.RED, setSide(robot, Side.LEFT));
        launchpad.bind(4, 0, Color.BLUE, setSide(robot, Side.RIGHT));
        final Command scoreCoral = robot.coralCommands.scoreCoral();
        launchpad.bind(7, 8, Launchpad.kScoreCoral, Command.noRequirements(coroutine -> {
            showSelectedLevel(robot, null);
            coroutine.await(scoreCoral);
        }).named("Launchpad Score Coral"));
        showSelectedLevel(robot, null);

        // Algae: pull the low or high algae off the closest face, then score it.
        launchpad.bind(1, 2, Launchpad.kAlgae, loadAlgae(robot, Level.L2));
        launchpad.bind(1, 3, Launchpad.kAlgae, loadAlgae(robot, Level.L3));
        launchpad.bind(1, 0, Launchpad.kAlgae, robot.algaeCommands.scoreAlgaeNet());
        launchpad.bind(2, 0, Launchpad.kAlgae, robot.algaeCommands.processorDelayed());

        // Coral intake.
        launchpad.bind(7, 1, Launchpad.kHumanPlayer, robot.coralCommands.intakeFromHumanPlayer());
        launchpad.bind(6, 1, Color.ORANGE_RED, robot.coralRoller.full());
        launchpad.bind(4, 3, Color.RED, robot.coralRoller.intake());
        launchpad.bind(5, 3, Color.WHITE, robot.coralCommands.holdToScore());
        launchpad.bind(6, 3, Color.PURPLE, robot.coralRoller.spit());

        // Algae intake.
        launchpad.bind(4, 4, Color.DARK_GREEN, robot.algaeIntake.intake());
        launchpad.bind(5, 4, Color.WHITE, robot.algaeIntake.stop());
        launchpad.bind(6, 4, Color.LIME_GREEN, robot.algaeIntake.outtake());

        // Manual elevator, unless already at the end of its travel.
        launchpad.bind(4, 5, Color.YELLOW, unless(robot.elevator::isAtMax, robot.elevator.setDutyCycle(0.5)));
        launchpad.bind(5, 5, Color.WHITE, robot.elevator.holdCurrent());
        launchpad.bind(6, 5, Color.CHOCOLATE, unless(robot.elevator::isAtMin, robot.elevator.setDutyCycle(-0.4)));

        // Arm presets, in degrees.
        launchpad.bind(6, 8, Color.GREEN, robot.superstructure.clearElevatorThen(robot.algaeArm.holdAt(Degrees.of(-20))));
        launchpad.bind(5, 8, Color.MEDIUM_PURPLE, robot.superstructure.clearElevatorThen(robot.coralArm.holdAt(Degrees.of(-20))));
        launchpad.bind(8, 2, Color.MEDIUM_PURPLE, robot.coralArm.moveTo(Degrees.of(-60)));
        launchpad.bind(8, 1, Color.LIME_GREEN, robot.algaeArm.moveTo(Degrees.of(-60)));
        launchpad.bind(7, 0, Color.MEDIUM_PURPLE, robot.coralArm.moveTo(Degrees.of(0)));
        launchpad.bind(8, 0, Color.GREEN, robot.algaeArm.moveTo(Degrees.of(0)));
        launchpad.bind(2, 8, Color.GREEN, robot.algaeArm.holdAt(AlgaeArmConstants.L34));
        launchpad.bind(2, 7, Color.GREEN, robot.algaeArm.holdAt(AlgaeArmConstants.L23));
        launchpad.bind(8, 3, Color.RED, robot.superstructure.fullyStowArms());

        // Target the closest branch and hold the coral arm and elevator at a level.
        launchpad.bind(8, 4, Color.PURPLE, holdCoralLevel(robot, Level.L4));
        launchpad.bind(8, 5, Color.PURPLE, holdCoralLevel(robot, Level.L3));
        launchpad.bind(8, 6, Color.PURPLE, holdCoralLevel(robot, Level.L2));
        launchpad.bind(8, 7, Color.PURPLE, holdCoralLevel(robot, Level.L1));
    }

    /** Run a command unless a condition is true when it starts. */
    private static Command unless(BooleanSupplier condition, Command command) {
        return Command.noRequirements(coroutine -> {
            if (!condition.getAsBoolean()) {
                coroutine.await(command);
            }
        }).named(command.name());
    }

    /**
     * Light the selected level in column 0, rows 5 and 6 (L4 and L3). The original also used rows 7
     * and 8 for L2 and L1, which overwrote the loaded indicators there, so those two levels only light
     * their own pad now.
     */
    private static void showSelectedLevel(Robot robot, Level level) {
        robot.launchpad.setColor(0, 5, level == Level.L4 ? Launchpad.kSelected : Launchpad.kNotSelected);
        robot.launchpad.setColor(0, 6, level == Level.L3 ? Launchpad.kSelected : Launchpad.kNotSelected);
    }
}
