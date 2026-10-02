// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import first.robot.Constants.DriveConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.Constants.ShooterConstants;
import first.robot.commands.SuperstructureCommands;
import first.robot.mechanisms.Agitator;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.Indexer;
import first.robot.mechanisms.IntakeArm;
import first.robot.mechanisms.IntakeRoller;
import first.robot.mechanisms.Kicker;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;
import first.robot.opmodes.auto.PathPlannerAuto;
import first.robot.pathplanner.EventTriggers;
import first.robot.pathplanner.NamedCommands;
import first.robot.pathplanner.AutoBuilder;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.telemetry.Telemetry;

/**
 * Holds the robot's mechanisms, controllers, and the commands shared by the opmodes in
 * {@code first.robot.opmodes}. OpModeRobot finds the {@code @Teleop} opmode in that package; every
 * PathPlanner auto in {@code deploy/pathplanner/autos} is added as an autonomous opmode here, along
 * with a mirrored copy of each (the original's "Flip Auto" chooser), which runs it on the other side
 * of the field's long centerline.
 */
public class Robot extends OpModeRobot {
    public final Swerve swerve = new Swerve();
    public final IntakeRoller intakeRoller = new IntakeRoller();
    public final IntakeArm intakeArm = new IntakeArm();
    public final Shooter shooter = new Shooter();
    public final Indexer indexer = new Indexer();
    public final Agitator agitator = new Agitator();
    public final Kicker kicker = new Kicker();
    public final Hood hood = new Hood();

    public final CommandNiDsXboxController driver = new CommandNiDsXboxController(OperatorConstants.kDriverControllerPort);
    public final CommandNiDsXboxController operator = new CommandNiDsXboxController(OperatorConstants.kOperatorControllerPort);

    /** Commands that use several mechanisms, shared by teleop and autonomous. */
    public final SuperstructureCommands superstructure =
        new SuperstructureCommands(swerve, shooter, kicker, indexer, agitator, hood, intakeRoller);
    private final HubTracker hubTracker = new HubTracker(driver.getNiDsXboxController());

    private final Scheduler scheduler = Scheduler.getDefault();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code. Default commands set here apply in every opmode.
     */
    public Robot() {
        swerve.setDefaultCommand(drive());
        shooter.setDefaultCommand(shooter.stop());
        kicker.setDefaultCommand(kicker.stop());
        indexer.setDefaultCommand(indexer.stop());
        agitator.setDefaultCommand(agitator.stop());
        intakeRoller.setDefaultCommand(intakeRoller.stop());
        hood.setDefaultCommand(hood.holdDown());
        intakeArm.setDefaultCommand(intakeArm.manual(operator::getLeftY, operator::getRightY));

        // Named commands and event triggers must exist before the autos are built.
        configurePathPlannerCommands();
        addAutos();
    }

    /** Drive with the driver's sticks, field relative from the driver's alliance wall. */
    public Command drive() {
        return swerve.drive(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(),
            DriveConstants.kTranslationScale, false, "Drive");
    }

    /** Drive slowly with the driver's sticks while facing the hub. */
    public Command aimAtHub() {
        return swerve.drive(() -> -driver.getLeftY(), () -> -driver.getLeftX(), () -> -driver.getRightX(),
            DriveConstants.kAimTranslationScale, true, "Aim at Hub");
    }

    /** The named commands and event markers the PathPlanner autos use. */
    private void configurePathPlannerCommands() {
        NamedCommands.registerCommand("ShootCommand",
            () -> superstructure.shootFromDistance().withTimeout(ShooterConstants.kAutoShotTimeout));
        // The sticks are idle in autonomous, so this only aims.
        NamedCommands.registerCommand("AimAtHub",
            () -> swerve.drive(() -> 0, () -> 0, () -> 0, DriveConstants.kAimTranslationScale, true, "Aim at Hub"));
        NamedCommands.registerCommand("ArmUp", intakeArm::wiggleUp);
        NamedCommands.registerCommand("ArmDown", intakeArm::wiggleDown);

        // Event markers run alongside the path. Intaking keeps going until IntakeStop, possibly in a
        // later path, or until a shot takes over the agitator.
        EventTriggers.onTrue("IntakeStart", superstructure.startIntake());
        EventTriggers.onTrue("IntakeStop", superstructure.stopIntake());
    }

    /**
     * Add an autonomous opmode for every PathPlanner auto, and a mirrored copy of each. Until commands
     * v3 support for PathPlanner is finished, they report an error and do nothing (see
     * {@link AutoBuilder}).
     */
    private void addAutos() {
        try {
            for (String autoName : AutoBuilder.getAllAutoNames()) {
                addOpMode(RobotMode.AUTONOMOUS, autoName, "PathPlanner",
                    () -> new PathPlannerAuto(autoName, false));
                addOpMode(RobotMode.AUTONOMOUS, autoName + " (Mirrored)", "PathPlanner Mirrored",
                    () -> new PathPlannerAuto(autoName, true));
            }
            publishOpModes();
        } catch (RuntimeException e) {
            DriverStationErrors.reportError("Could not add the PathPlanner autos: " + e, e.getStackTrace());
        }
    }

    /**
     * This function is called every 20 ms, no matter the mode. The mechanism updates run first, as
     * v2 subsystem periodic() methods did, then the scheduler.
     */
    @Override
    public void robotPeriodic() {
        swerve.periodic();
        intakeRoller.periodic();
        intakeArm.periodic();
        shooter.periodic();
        indexer.periodic();
        agitator.periodic();
        kicker.periodic();
        hood.periodic();
        hubTracker.update();

        scheduler.run();
        Telemetry.log("Scheduler", scheduler, Scheduler.proto);
    }

    @Override
    public void simulationPeriodic() {
        swerve.simulationPeriodic();
        intakeRoller.simulationPeriodic();
        intakeArm.simulationPeriodic();
        shooter.simulationPeriodic();
        indexer.simulationPeriodic();
        agitator.simulationPeriodic();
        kicker.simulationPeriodic();
        hood.simulationPeriodic();
    }
}
