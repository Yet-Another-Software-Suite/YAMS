// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2026.

package first.robot;

import first.robot.Constants.OperatorConstants;
import first.robot.commands.SuperstructureCommands;
import first.robot.mechanisms.Agitator;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.Indexer;
import first.robot.mechanisms.IntakeArm;
import first.robot.mechanisms.IntakeRoller;
import first.robot.mechanisms.Kicker;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;
import first.robot.opmodes.auto.Auto2PreloadAuto;
import first.robot.opmodes.auto.Auto3WithWiggleAuto;
import first.robot.opmodes.auto.AutoOneAdvancedAuto;
import first.robot.opmodes.auto.AutoOneAuto;
import first.robot.opmodes.auto.AutoThreeAdvancedAuto;
import first.robot.opmodes.auto.AutoThreeAdvancedMirrorAuto;
import first.robot.opmodes.auto.AutoThreeAuto;
import first.robot.opmodes.auto.AutoTwoAuto;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.telemetry.Telemetry;

/**
 * Holds the robot's mechanisms, controllers, and the commands shared by the opmodes in
 * {@code first.robot.opmodes}. OpModeRobot finds the {@code @Teleop} and {@code @Autonomous} opmodes
 * in that package; a mirrored copy of each auto (the original's "Flip Auto" chooser), which runs it on
 * the other side of the field's long centerline, is added here.
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
        // Hold still unless an opmode says otherwise. The teleop opmodes replace this with stick driving, so autos
        // never read the sticks.
        swerve.setDefaultCommand(swerve.driveFieldRelative(ChassisVelocities::new));
        shooter.setDefaultCommand(shooter.stop());
        kicker.setDefaultCommand(kicker.stop());
        indexer.setDefaultCommand(indexer.stop());
        agitator.setDefaultCommand(agitator.stop());
        intakeRoller.setDefaultCommand(intakeRoller.stop());
        hood.setDefaultCommand(hood.holdDown());
        intakeArm.setDefaultCommand(intakeArm.manual(operator::getLeftY, operator::getRightY));

        addMirroredAutos();
    }

    /** Add a mirrored copy of every auto, which runs it on the other side of the field's long centerline. */
    private void addMirroredAutos() {
        addOpMode(RobotMode.AUTONOMOUS, "Auto 2 Preload (Mirrored)", "Mirrored", () -> new Auto2PreloadAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto 3 With Wiggle (Mirrored)", "Mirrored",
            () -> new Auto3WithWiggleAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto One (Mirrored)", "Mirrored", () -> new AutoOneAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto One Advanced (Mirrored)", "Mirrored",
            () -> new AutoOneAdvancedAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto Three (Mirrored)", "Mirrored", () -> new AutoThreeAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto Three Advanced (Mirrored)", "Mirrored",
            () -> new AutoThreeAdvancedAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto Three Advanced Mirror (Mirrored)", "Mirrored",
            () -> new AutoThreeAdvancedMirrorAuto(this, true));
        addOpMode(RobotMode.AUTONOMOUS, "Auto Two (Mirrored)", "Mirrored", () -> new AutoTwoAuto(this, true));
        publishOpModes();
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
