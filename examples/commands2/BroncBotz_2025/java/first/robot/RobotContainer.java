// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot;

import static org.wpilib.units.Units.Degrees;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.CoralIntakeConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.commands.Autos;
import first.robot.commands.SuperstructureCommands;
import first.robot.subsystems.AlgaeArm;
import first.robot.subsystems.AlgaeIntake;
import first.robot.subsystems.CoralArm;
import first.robot.subsystems.CoralIntake;
import first.robot.subsystems.Elevator;
import first.robot.subsystems.Swerve;
import first.robot.util.Launchpad;
import first.robot.util.ReefTargeting;
import first.robot.util.ReefTargeting.Branch;
import first.robot.util.ReefTargeting.Level;
import first.robot.util.ReefTargeting.Side;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.Trigger;
import org.wpilib.framework.RobotBase;
import org.wpilib.util.Color;
import yams.commands2.swerve.SwerveInputStream;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Elevator elevator = new Elevator();
    private final CoralArm coralArm = new CoralArm();
    private final CoralIntake coralIntake = new CoralIntake();
    private final AlgaeArm algaeArm = new AlgaeArm();
    private final AlgaeIntake algaeIntake = new AlgaeIntake();
    private final ReefTargeting targeting = new ReefTargeting();

    private final CommandNiDsXboxController driver = new CommandNiDsXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandNiDsXboxController operator = new CommandNiDsXboxController(OperatorConstants.kOperatorControllerPort);
    private final Launchpad launchpad = new Launchpad(OperatorConstants.kLaunchpadPort1, OperatorConstants.kLaunchpadPort2,
        OperatorConstants.kLaunchpadPort3, Launchpad.kPressed);

    private final SuperstructureCommands superstructure =
        new SuperstructureCommands(swerve, elevator, coralArm, coralIntake, algaeArm, algaeIntake, targeting);

    // Off by default, as in the original; X turns it on and Y off.
    private boolean allianceRelative = false;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();
        configureLaunchpadBindings();
        if (RobotBase.isSimulation()) {
            configureSimulatedGamePieces();
        }
    }

    private void configureDefaultCommands() {
        // The sticks drive relative to the field, slower while the left bumper is held.
        final SwerveInputStream normal = swerve.createDriverInput(() -> -driver.getLeftY(), () -> -driver.getLeftX(),
                () -> -driver.getRightX(), OperatorConstants.kTranslationScale, () -> allianceRelative)
            .withTelemetry("Driver", TelemetryVerbosity.HIGH);
        final SwerveInputStream slow = swerve.createDriverInput(() -> -driver.getLeftY(), () -> -driver.getLeftX(),
                () -> -driver.getRightX(), OperatorConstants.kSlowTranslationScale, () -> allianceRelative)
            .withTelemetry("Driver Slow", TelemetryVerbosity.HIGH);
        // SwerveInputStream output is field relative.
        swerve.setDefaultCommand(swerve.driveFieldRelative(
            () -> driver.leftBumper().getAsBoolean() ? slow.get() : normal.get()));

        // Return the elevator to the bottom and hold the arms where they are.
        elevator.setDefaultCommand(elevator.holdAt(Constants.ElevatorConstants.kMinHeight));
        coralArm.setDefaultCommand(coralArm.holdCurrent());
        algaeArm.setDefaultCommand(algaeArm.holdCurrent());
        // Gently hold any game piece; the algae only while its arm is raised.
        coralIntake.setDefaultCommand(coralIntake.hold(coralArm::isCoralLoaded));
        algaeIntake.setDefaultCommand(algaeIntake.hold(
            () -> algaeArm.isAlgaeLoaded() && algaeArm.getAngle().gte(AlgaeArmConstants.kHoldAboveAngle)));
    }

    private void configureDriverBindings() {
        // Re-read the absolute encoders and swing the arm out.
        driver.b().whileTrue(Commands.runOnce(coralArm::synchronizeAbsoluteEncoder)
            .andThen(coralArm.moveTo(CoralArmConstants.kStowed)));
        driver.a().whileTrue(Commands.runOnce(algaeArm::synchronizeAbsoluteEncoder)
            .andThen(algaeArm.moveTo(AlgaeArmConstants.kStowed)));
        driver.x().onTrue(Commands.runOnce(() -> allianceRelative = true));
        driver.y().onTrue(Commands.runOnce(() -> allianceRelative = false));
    }

    private void configureOperatorBindings() {
        operator.a().whileTrue(superstructure.coralLevel(Level.L1, CoralIntakeConstants.kRest));
        operator.b().whileTrue(superstructure.coralLevel(Level.L2, CoralIntakeConstants.kActive));
        operator.x().whileTrue(superstructure.coralLevel(Level.L3, CoralIntakeConstants.kActive));
        operator.y().whileTrue(superstructure.coralLevel(Level.L4, CoralIntakeConstants.kActive));
        operator.leftBumper().whileTrue(coralIntake.outtake());
        operator.rightBumper().whileTrue(algaeIntake.outtake());
        operator.rightTrigger().whileTrue(algaeIntake.intake());
        operator.leftTrigger().whileTrue(superstructure.intakeFromHumanPlayer());
        // The D-pad triggers live on the generic HID in 2027.
        operator.getHID().povLeft().whileTrue(superstructure.scoreAlgaeNet());
        operator.getHID().povRight().whileTrue(algaeArm.moveTo(AlgaeArmConstants.PROCESSOR).alongWith(algaeIntake.outtake()));
        operator.getHID().povDown().whileTrue(superstructure.algaeReefPosition(Level.L2));
        operator.getHID().povUp().whileTrue(superstructure.algaeReefPosition(Level.L3));
        operator.start().whileTrue(superstructure.stowArms());
    }

    /** Pick a coral level, target the closest branch, and highlight the level. */
    private Command selectCoralLevel(Level level) {
        return Commands.runOnce(() -> {
            targeting.targetClosestBranch(swerve.getPose());
            targeting.setLevel(level);
            showSelectedLevel(level);
        });
    }

    private void configureLaunchpadBindings() {
        // Coral: pick the level (targets the closest branch), the side, then score.
        launchpad.bind(0, 1, Launchpad.kCoralLevel, selectCoralLevel(Level.L4));
        launchpad.bind(0, 2, Launchpad.kCoralLevel, selectCoralLevel(Level.L3));
        launchpad.bind(0, 3, Launchpad.kCoralLevel, selectCoralLevel(Level.L2));
        launchpad.bind(0, 4, Launchpad.kCoralLevel, selectCoralLevel(Level.L1));
        launchpad.bind(3, 0, Color.RED, Commands.runOnce(() -> targeting.setSide(Side.LEFT)));
        launchpad.bind(4, 0, Color.BLUE, Commands.runOnce(() -> targeting.setSide(Side.RIGHT)));
        launchpad.bind(7, 8, Launchpad.kScoreCoral,
            Commands.runOnce(() -> showSelectedLevel(null)).andThen(superstructure.scoreCoral()));
        showSelectedLevel(null);

        // Algae: pull the low or high algae off the closest face, then score it.
        launchpad.bind(1, 2, Launchpad.kAlgae, Commands.runOnce(() -> {
            targeting.setLevel(Level.L2);
            targeting.targetClosestBranch(swerve.getPose());
        }).andThen(superstructure.loadAlgae()));
        launchpad.bind(1, 3, Launchpad.kAlgae, Commands.runOnce(() -> {
            targeting.targetClosestBranch(swerve.getPose());
            targeting.setLevel(Level.L3);
        }).andThen(superstructure.loadAlgae()));
        launchpad.bind(1, 0, Launchpad.kAlgae, superstructure.scoreAlgaeNet());
        launchpad.bind(2, 0, Launchpad.kAlgae, algaeArm.holdAt(AlgaeArmConstants.PROCESSOR)
            .alongWith(Commands.waitSeconds(0.3).andThen(algaeIntake.outtake())));

        // Coral intake.
        launchpad.bind(7, 1, Launchpad.kHumanPlayer, superstructure.intakeFromHumanPlayer());
        launchpad.bind(6, 1, Color.ORANGE_RED, coralIntake.rollerFull());
        launchpad.bind(4, 3, Color.RED, coralIntake.intake());
        launchpad.bind(5, 3, Color.WHITE, coralIntake.score());
        launchpad.bind(6, 3, Color.PURPLE, coralIntake.spit());

        // Algae intake.
        launchpad.bind(4, 4, Color.DARK_GREEN, algaeIntake.intake());
        launchpad.bind(5, 4, Color.WHITE, algaeIntake.stop());
        launchpad.bind(6, 4, Color.LIME_GREEN, algaeIntake.outtake());

        // Manual elevator.
        launchpad.bind(4, 5, Color.YELLOW, elevator.setDutyCycle(0.5).unless(elevator.atMax));
        launchpad.bind(5, 5, Color.WHITE, elevator.holdCurrent());
        launchpad.bind(6, 5, Color.CHOCOLATE, elevator.setDutyCycle(-0.4).unless(elevator.atMin));

        // Arm presets, in degrees.
        launchpad.bind(6, 8, Color.GREEN, superstructure.clearElevatorThen(algaeArm.holdAt(Degrees.of(-20))));
        launchpad.bind(5, 8, Color.MEDIUM_PURPLE, superstructure.clearElevatorThen(coralArm.holdAt(Degrees.of(-20))));
        launchpad.bind(8, 2, Color.MEDIUM_PURPLE, coralArm.moveTo(Degrees.of(-60)));
        launchpad.bind(8, 1, Color.LIME_GREEN, algaeArm.moveTo(Degrees.of(-60)));
        launchpad.bind(7, 0, Color.MEDIUM_PURPLE, coralArm.moveTo(Degrees.of(0)));
        launchpad.bind(8, 0, Color.GREEN, algaeArm.moveTo(Degrees.of(0)));
        launchpad.bind(2, 8, Color.GREEN, algaeArm.holdAt(AlgaeArmConstants.L34));
        launchpad.bind(2, 7, Color.GREEN, algaeArm.holdAt(AlgaeArmConstants.L23));
        launchpad.bind(8, 3, Color.RED, algaeArm.holdAt(AlgaeArmConstants.kFullyStowed)
            .alongWith(coralArm.holdAt(CoralArmConstants.kFullyStowed)));

        // Target the closest branch and hold the coral arm and elevator at a level.
        launchpad.bind(8, 4, Color.PURPLE, holdCoralLevel(Level.L4));
        launchpad.bind(8, 5, Color.PURPLE, holdCoralLevel(Level.L3));
        launchpad.bind(8, 6, Color.PURPLE, holdCoralLevel(Level.L2));
        launchpad.bind(8, 7, Color.PURPLE, holdCoralLevel(Level.L1));

        // Loaded indicators.
        launchpad.setColor(0, 8, Launchpad.kUnloaded);
        launchpad.setColor(0, 7, Launchpad.kUnloaded);
        coralArm.coralLoaded
            .onTrue(Commands.runOnce(() -> launchpad.setColor(0, 8, Launchpad.kCoralLoaded)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> launchpad.setColor(0, 8, Launchpad.kUnloaded)).ignoringDisable(true));
        algaeArm.algaeLoaded
            .onTrue(Commands.runOnce(() -> launchpad.setColor(0, 7, Launchpad.kAlgaeLoaded)).ignoringDisable(true))
            .onFalse(Commands.runOnce(() -> launchpad.setColor(0, 7, Launchpad.kUnloaded)).ignoringDisable(true));
    }

    private Command holdCoralLevel(Level level) {
        return Commands.runOnce(() -> targeting.targetClosestBranch(swerve.getPose()))
            .andThen(coralArm.holdAt(CoralArm.coralAngle(level)).alongWith(elevator.holdCoralLevel(level)));
    }

    /**
     * Light the selected level in column 0, rows 5 and 6 (L4 and L3). The original also used rows 7
     * and 8 for L2 and L1, which overwrote the loaded indicators there, so those two levels only light
     * their own pad now.
     */
    private void showSelectedLevel(Level level) {
        launchpad.setColor(0, 5, level == Level.L4 ? Launchpad.kSelected : Launchpad.kNotSelected);
        launchpad.setColor(0, 6, level == Level.L3 ? Launchpad.kSelected : Launchpad.kNotSelected);
    }

    /**
     * The simulator has no game pieces, so load a coral after half a second of intaking at the human
     * player station and an algae after half a second of intaking, and drop them when spat out. A
     * coral also comes off onto the branch when the coral arm swings down onto it at the scoring pose.
     * The game pieces only set what the simulated sensors read; the robot code reads the sensors as
     * it would on the robot.
     */
    private void configureSimulatedGamePieces() {
        new Trigger(() -> coralArm.isCoralLoaded() && elevator.isAtCoralLevel(targeting.getLevel())
            && targeting.getCoralScoringPose().map(pose -> swerve.getPose().getTranslation().getDistance(pose.getTranslation()) < 0.15).orElse(false)
            && coralArm.getAngle().lt(CoralArm.coralAngle(targeting.getLevel()).minus(CoralArmConstants.kScoreDrop.div(2))))
            .onTrue(Commands.runOnce(() -> coralArm.setSimCoralLoaded(false)));
        new Trigger(() -> coralIntake.getRollerDutyCycle() > 0.3 && coralArm.isNear(CoralArmConstants.HP))
            .debounce(0.5)
            .onTrue(Commands.runOnce(() -> coralArm.setSimCoralLoaded(true)));
        new Trigger(() -> coralIntake.getRollerDutyCycle() < -0.3)
            .debounce(0.25)
            .onTrue(Commands.runOnce(() -> coralArm.setSimCoralLoaded(false)));
        new Trigger(() -> algaeIntake.getDutyCycle() > 0.5)
            .debounce(0.5)
            .onTrue(Commands.runOnce(() -> algaeArm.setSimAlgaeLoaded(true)));
        new Trigger(() -> algaeIntake.getDutyCycle() < -0.5)
            .debounce(0.25)
            .onTrue(Commands.runOnce(() -> algaeArm.setSimAlgaeLoaded(false)));
    }

    /** @return The drivetrain, for tests. */
    Swerve getSwerve() {
        return swerve;
    }

    /** @return The coral arm, for tests. */
    CoralArm getCoralArm() {
        return coralArm;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return Autos.coralL4(Branch.H, targeting, elevator, coralArm, superstructure);
    }
}
