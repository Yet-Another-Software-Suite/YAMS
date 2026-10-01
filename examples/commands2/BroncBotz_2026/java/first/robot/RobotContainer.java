// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import static first.robot.Constants.FeedConstants.kAgitatorBackOff;
import static first.robot.Constants.FeedConstants.kAgitatorFeed;
import static first.robot.Constants.IntakeConstants.kRollerIntake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import first.robot.Constants.DriveConstants;
import first.robot.Constants.HoodConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.Constants.ShooterConstants;
import first.robot.commands.SuperstructureCommands;
import first.robot.subsystems.Agitator;
import first.robot.subsystems.Hood;
import first.robot.subsystems.Indexer;
import first.robot.subsystems.IntakeArm;
import first.robot.subsystems.IntakeRoller;
import first.robot.subsystems.Kicker;
import first.robot.subsystems.Shooter;
import first.robot.subsystems.Swerve;
import java.util.stream.Stream;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.tunable.Selectable;
import org.wpilib.tunable.Tunables;
import yams.commands2.swerve.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final IntakeRoller intakeRoller = new IntakeRoller();
    private final IntakeArm intakeArm = new IntakeArm();
    private final Shooter shooter = new Shooter();
    private final Indexer indexer = new Indexer();
    private final Agitator agitator = new Agitator();
    private final Kicker kicker = new Kicker();
    private final Hood hood = new Hood();

    private final CommandNiDsXboxController driver = new CommandNiDsXboxController(OperatorConstants.kDriverControllerPort);
    private final CommandNiDsXboxController operator = new CommandNiDsXboxController(OperatorConstants.kOperatorControllerPort);

    private final SuperstructureCommands superstructure =
        new SuperstructureCommands(swerve, shooter, kicker, indexer, agitator, hood, intakeRoller);
    private final HubTracker hubTracker = new HubTracker(driver.getNiDsXboxController());

    private final Selectable<Command> autoChooser;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Named commands and event triggers must exist before the autos are loaded.
        configurePathPlannerCommands();
        configureDefaultCommands();
        configureDriverBindings();
        configureOperatorBindings();

        // Every auto in deploy/pathplanner/autos, plus a mirrored copy of each (the original's "Flip
        // Auto" chooser), which runs it on the other side of the field's long centerline.
        autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(autos -> autos.flatMap(auto -> {
            final PathPlannerAuto mirrored = new PathPlannerAuto(auto.getName(), true);
            mirrored.setName(auto.getName() + " (Mirrored)");
            return Stream.of(auto, mirrored);
        }));
        autoChooser.addDefault("Do Nothing", Commands.none());
        Tunables.publish("Auto Chooser", autoChooser);
    }

    /** Driver input stream: field relative from the driver's alliance wall. */
    private SwerveInputStream driverInput() {
        return swerve.createDriverInput(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX());
    }

    /** Drive slowly with the driver's sticks while facing the hub. */
    private Command aimAtHub(SwerveInputStream input) {
        return swerve.driveFieldOriented(input
            .withScaleTranslation(DriveConstants.kAimTranslationScale)
            .withAim(() -> new Pose2d(Field.hub(), Rotation2d.ZERO), () -> true))
            .withName("Aim at Hub");
    }

    /** The named commands and event markers the PathPlanner autos use. */
    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("ShootCommand", superstructure.shootFromDistance().withTimeout(ShooterConstants.kAutoShotTimeout));
        // The sticks are idle in autonomous, so this only aims.
        NamedCommands.registerCommand("AimAtHub", aimAtHub(swerve.createDriverInput(() -> 0, () -> 0, () -> 0)));
        NamedCommands.registerCommand("ArmUp", intakeArm.wiggleUp());
        NamedCommands.registerCommand("ArmDown", intakeArm.wiggleDown());

        // Event markers run alongside the path. Intaking keeps going until IntakeStop, possibly in a
        // later path, or until a shot takes over the agitator.
        new EventTrigger("IntakeStart").onTrue(Commands.parallel(intakeRoller.runAt(kRollerIntake), agitator.runAt(kAgitatorFeed)));
        new EventTrigger("IntakeStop").onTrue(Commands.parallel(intakeRoller.stop(), agitator.runAt(kAgitatorFeed)));
    }

    private void configureDefaultCommands() {
        swerve.setDefaultCommand(swerve.driveFieldOriented(driverInput()));
        shooter.setDefaultCommand(shooter.stop());
        kicker.setDefaultCommand(kicker.stop());
        indexer.setDefaultCommand(indexer.stop());
        agitator.setDefaultCommand(agitator.stop());
        intakeRoller.setDefaultCommand(intakeRoller.stop());
        hood.setDefaultCommand(hood.holdAt(HoodConstants.kDown));
        intakeArm.setDefaultCommand(intakeArm.manual(operator::getLeftY, operator::getRightY));
    }

    private void configureDriverBindings() {
        driver.a().whileTrue(aimAtHub(driverInput()));
        driver.rightBumper().whileTrue(swerve.driveFieldOriented(driverInput().withScaleTranslation(DriveConstants.kSlowModeTranslationScale))
            .withName("Slow Mode"));
        driver.leftBumper().whileTrue(swerve.lockPose());
        driver.start().and(driver.back()).onTrue(swerve.zeroGyroWithAlliance());
        driver.x().whileTrue(agitator.runAt(kAgitatorBackOff));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(swerve.resetOdometryCommand(Field.kBlueRobotPoseAtHub));
    }

    private void configureOperatorBindings() {
        operator.getHID().povRight().whileTrue(superstructure.shootAt(ShooterConstants.kCloseShot));
        operator.x().whileTrue(superstructure.shootAt(ShooterConstants.kMidShot));
        operator.y().whileTrue(superstructure.shootAt(ShooterConstants.kFarShot));
        operator.rightTrigger(0.2).whileTrue(superstructure.shootFromDistance());
        operator.getHID().povLeft().whileTrue(superstructure.pass());

        operator.leftTrigger(0.3).whileTrue(superstructure.intake());
        operator.b().whileTrue(superstructure.outtake());
        operator.a().whileTrue(superstructure.unjam());

        operator.leftBumper().onTrue(intakeArm.toIntake());
        operator.rightBumper().whileTrue(intakeArm.agitate());
        operator.getHID().povDown().onTrue(intakeArm.toDown());
        operator.getHID().povUp().onTrue(intakeArm.toUp());
        operator.start().and(operator.back()).onTrue(intakeArm.resetEncoder());
    }

    /** @return The autonomous command selected on the dashboard. */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /** Update the hub tracker. Call once per loop. */
    public void periodic() {
        hubTracker.update();
    }
}
