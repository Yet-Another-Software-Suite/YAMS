// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot;

import first.robot.commands.AutoRoutines;
import first.robot.commands.ManualDrive;
import first.robot.commands.SubsystemCommands;
import first.robot.subsystems.Feeder;
import first.robot.subsystems.Floor;
import first.robot.subsystems.Hanger;
import first.robot.subsystems.Hood;
import first.robot.subsystems.Intake;
import first.robot.subsystems.Limelight;
import first.robot.subsystems.Shooter;
import first.robot.subsystems.Swerve;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final Swerve swerve = new Swerve();
    private final Intake intake = new Intake();
    private final Floor floor = new Floor();
    private final Feeder feeder = new Feeder();
    private final Shooter shooter = new Shooter();
    private final Hood hood = new Hood();
    private final Hanger hanger = new Hanger();
    private final Limelight limelight = new Limelight("limelight");

    private final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

    private final AutoRoutines autoRoutines = new AutoRoutines(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        limelight
    );

    private final SubsystemCommands subsystemCommands = new SubsystemCommands(
        swerve,
        intake,
        floor,
        feeder,
        shooter,
        hood,
        hanger,
        () -> -driver.getLeftY(),
        () -> -driver.getLeftX()
    );

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        configureBindings();
        autoRoutines.configure();
        // Swerve telemetry is published by the YAMS SwerveDrive, replacing SwerveTelemetry.
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * org.wpilib.command2.button.CommandGenericHID}'s subclasses for {@link
     * CommandNiDsXboxController Xbox}/{@link org.wpilib.command2.button.CommandPS4Controller
     * PS4} controllers or {@link org.wpilib.command2.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        configureManualDriveBindings();
        limelight.setDefaultCommand(updateVisionCommand());
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop())
            .onTrue(intake.homingCommand())
            .onTrue(hanger.homingCommand());

        driver.rightTrigger().whileTrue(subsystemCommands.aimAndShoot());
        driver.rightBumper().whileTrue(subsystemCommands.shootManually());
        driver.leftTrigger().whileTrue(intake.intakeCommand());
        driver.leftBumper().onTrue(intake.runOnce(() -> intake.set(Intake.Position.STOWED)));
        // The D-pad triggers live on the generic HID in 2027.
        driver.getHID().povUp().onTrue(hanger.positionCommand(Hanger.Position.HANGING));
        driver.getHID().povDown().onTrue(hanger.positionCommand(Hanger.Position.HUNG));
    }

    private void configureManualDriveBindings() {
        final ManualDrive manualDrive = new ManualDrive(
            swerve,
            () -> -driver.getLeftY(),
            () -> -driver.getLeftX(),
            () -> -driver.getRightX()
        );
        swerve.setDefaultCommand(manualDrive.command());
        driver.a().onTrue(Commands.runOnce(() -> manualDrive.setLockedHeading(Rotation2d.k180deg)));
        driver.b().onTrue(Commands.runOnce(() -> manualDrive.setLockedHeading(Rotation2d.CW_90DEG)));
        driver.x().onTrue(Commands.runOnce(() -> manualDrive.setLockedHeading(Rotation2d.CCW_90DEG)));
        driver.y().onTrue(Commands.runOnce(() -> manualDrive.setLockedHeading(Rotation2d.ZERO)));
        driver.back().onTrue(Commands.runOnce(() -> manualDrive.seedFieldCentric()));
    }

    private Command updateVisionCommand() {
        return limelight.run(() -> {
            final Pose2d currentRobotPose = swerve.getPose();
            final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
            measurement.ifPresent(m -> {
                swerve.addVisionMeasurement(
                    m.poseEstimate.pose,
                    m.poseEstimate.timestampSeconds,
                    m.standardDeviations
                );
            });
        })
        .ignoringDisable(true);
    }
}
