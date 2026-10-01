// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot.opmodes.teleop;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.CoralIntakeConstants;
import first.robot.Robot;
import first.robot.util.Launchpad;
import first.robot.util.ReefTargeting.Level;
import first.robot.util.ReefTargeting.Side;
import java.util.function.BooleanSupplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.opmode.OpMode;
import org.wpilib.opmode.Teleop;
import org.wpilib.util.Color;

/**
 * Teleop with the original's driver, operator and Launchpad bindings. They are created in the
 * constructor, so they only exist while this opmode is selected. Driving, including the slow mode and
 * the alliance relative toggle, is the drivetrain's default command, set in {@link Robot}.
 */
@Teleop(name = "Driver Teleop")
public class DriverTeleop implements OpMode {
    private final Robot robot;

    /**
     * Creates the teleop opmode. The OpModeRobot framework calls this when the opmode is selected on
     * the driver station.
     *
     * @param robot The robot instance to control.
     */
    public DriverTeleop(Robot robot) {
        this.robot = robot;
        configureDriverBindings(robot.driver);
        configureOperatorBindings(robot.operator);
        configureLaunchpadBindings(robot.launchpad);
    }

    private void configureDriverBindings(CommandNiDsXboxController driver) {
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

    private void configureOperatorBindings(CommandNiDsXboxController operator) {
        operator.a().whileTrue(robot.superstructure.coralLevel(Level.L1, CoralIntakeConstants.kRest));
        operator.b().whileTrue(robot.superstructure.coralLevel(Level.L2, CoralIntakeConstants.kActive));
        operator.x().whileTrue(robot.superstructure.coralLevel(Level.L3, CoralIntakeConstants.kActive));
        operator.y().whileTrue(robot.superstructure.coralLevel(Level.L4, CoralIntakeConstants.kActive));
        operator.leftBumper().whileTrue(robot.coralIntake.outtake());
        operator.rightBumper().whileTrue(robot.algaeIntake.outtake());
        operator.rightTrigger().whileTrue(robot.algaeIntake.intake());
        operator.leftTrigger().whileTrue(robot.superstructure.intakeFromHumanPlayer());
        // The D-pad triggers live on the generic HID in 2027.
        operator.getHID().povLeft().whileTrue(robot.superstructure.scoreAlgaeNet());
        operator.getHID().povRight().whileTrue(robot.superstructure.processor());
        operator.getHID().povDown().whileTrue(robot.superstructure.algaeReefPosition(Level.L2));
        operator.getHID().povUp().whileTrue(robot.superstructure.algaeReefPosition(Level.L3));
        operator.start().whileTrue(robot.superstructure.stowArms());
    }

    /** Pick a coral level, target the closest branch, and highlight the level. */
    private Command selectCoralLevel(Level level) {
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            robot.targeting.setLevel(level);
            showSelectedLevel(level);
        }).named("Select " + level);
    }

    /** Target the closest reef face's algae at a level, then pull it off. */
    private Command loadAlgae(Level level) {
        final Command load = robot.superstructure.loadAlgae();
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            robot.targeting.setLevel(level);
            coroutine.await(load);
        }).named("Load Algae " + level);
    }

    /** Target the closest branch, then hold the coral arm and elevator at a level. */
    private Command holdCoralLevel(Level level) {
        final Command hold = robot.superstructure.holdCoralLevel(level);
        return Command.noRequirements(coroutine -> {
            robot.targeting.targetClosestBranch(robot.swerve.getPose());
            coroutine.await(hold);
        }).named("Target and Hold " + level);
    }

    private Command setSide(Side side) {
        return Command.noRequirements(coroutine -> robot.targeting.setSide(side)).named("Side " + side);
    }

    private void configureLaunchpadBindings(Launchpad launchpad) {
        // Coral: pick the level (targets the closest branch), the side, then score.
        launchpad.bind(0, 1, Launchpad.kCoralLevel, selectCoralLevel(Level.L4));
        launchpad.bind(0, 2, Launchpad.kCoralLevel, selectCoralLevel(Level.L3));
        launchpad.bind(0, 3, Launchpad.kCoralLevel, selectCoralLevel(Level.L2));
        launchpad.bind(0, 4, Launchpad.kCoralLevel, selectCoralLevel(Level.L1));
        launchpad.bind(3, 0, Color.RED, setSide(Side.LEFT));
        launchpad.bind(4, 0, Color.BLUE, setSide(Side.RIGHT));
        final Command scoreCoral = robot.superstructure.scoreCoral();
        launchpad.bind(7, 8, Launchpad.kScoreCoral, Command.noRequirements(coroutine -> {
            showSelectedLevel(null);
            coroutine.await(scoreCoral);
        }).named("Launchpad Score Coral"));
        showSelectedLevel(null);

        // Algae: pull the low or high algae off the closest face, then score it.
        launchpad.bind(1, 2, Launchpad.kAlgae, loadAlgae(Level.L2));
        launchpad.bind(1, 3, Launchpad.kAlgae, loadAlgae(Level.L3));
        launchpad.bind(1, 0, Launchpad.kAlgae, robot.superstructure.scoreAlgaeNet());
        launchpad.bind(2, 0, Launchpad.kAlgae, robot.superstructure.processorDelayed());

        // Coral intake.
        launchpad.bind(7, 1, Launchpad.kHumanPlayer, robot.superstructure.intakeFromHumanPlayer());
        launchpad.bind(6, 1, Color.ORANGE_RED, robot.coralIntake.rollerFull());
        launchpad.bind(4, 3, Color.RED, robot.coralIntake.intake());
        launchpad.bind(5, 3, Color.WHITE, robot.coralIntake.score());
        launchpad.bind(6, 3, Color.PURPLE, robot.coralIntake.spit());

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
        launchpad.bind(8, 4, Color.PURPLE, holdCoralLevel(Level.L4));
        launchpad.bind(8, 5, Color.PURPLE, holdCoralLevel(Level.L3));
        launchpad.bind(8, 6, Color.PURPLE, holdCoralLevel(Level.L2));
        launchpad.bind(8, 7, Color.PURPLE, holdCoralLevel(Level.L1));
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
    private void showSelectedLevel(Level level) {
        robot.launchpad.setColor(0, 5, level == Level.L4 ? Launchpad.kSelected : Launchpad.kNotSelected);
        robot.launchpad.setColor(0, 6, level == Level.L3 ? Launchpad.kSelected : Launchpad.kNotSelected);
    }
}
