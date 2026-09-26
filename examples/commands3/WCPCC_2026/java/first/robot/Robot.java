// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from the WCP 2026 Competitive Concept (MIT, see LICENSE-WCP).

package first.robot;

import static org.wpilib.units.Units.Volts;

import first.robot.commands.Drive;
import first.robot.commands.MechanismCommands;
import first.robot.mechanisms.Feeder;
import first.robot.mechanisms.Floor;
import first.robot.mechanisms.Hanger;
import first.robot.mechanisms.Hood;
import first.robot.mechanisms.IntakePivot;
import first.robot.mechanisms.IntakeRollers;
import first.robot.mechanisms.Limelight;
import first.robot.mechanisms.Shooter;
import first.robot.mechanisms.Swerve;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.RobotModeTriggers;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.system.RobotController;
import org.wpilib.telemetry.Telemetry;

/**
 * Holds the robot's mechanisms, the driver controller, and the commands shared by the opmodes in
 * {@code first.robot.opmodes}. OpModeRobot finds the {@code @Teleop} and {@code @Autonomous}
 * opmodes in that package and constructs the one selected on the driver station.
 */
public class Robot extends OpModeRobot {
    public final Swerve swerve = new Swerve();
    public final IntakePivot intakePivot = new IntakePivot();
    public final IntakeRollers intakeRollers = new IntakeRollers();
    public final Floor floor = new Floor();
    public final Feeder feeder = new Feeder();
    public final Shooter shooter = new Shooter();
    public final Hood hood = new Hood();
    public final Hanger hanger = new Hanger();
    public final Limelight limelight = new Limelight("limelight");

    public final CommandNiDsXboxController driver = new CommandNiDsXboxController(0);

    /** Commands that use several mechanisms, shared by teleop and autonomous. */
    public final MechanismCommands mechanismCommands = new MechanismCommands(
        swerve,
        intakePivot,
        intakeRollers,
        floor,
        feeder,
        shooter,
        hood,
        hanger
    );

    private final Scheduler scheduler = Scheduler.getDefault();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code. Default commands set here apply in every opmode.
     */
    public Robot() {
        // Manual drive is the default in every mode, as in the v2 port, so the drivetrain holds its
        // heading between autonomous trajectories.
        swerve.setDefaultCommand(Drive.teleop(swerve, driver));
        // Lowest priority so an autonomous routine can pause vision with limelight.idle().
        limelight.setDefaultCommand(updateVisionCommand());
        // The rollers stop whenever no command is using them, at the lowest priority.
        feeder.setDefaultCommand(feeder.stop());
        floor.setDefaultCommand(floor.stop());
        intakeRollers.setDefaultCommand(intakeRollers.stop());
        // Home when autonomous or teleop is enabled, as in the v2 port. Disabling cancels it.
        RobotModeTriggers.autonomous().or(RobotModeTriggers.teleop()).whileTrue(mechanismCommands.home());
        // 2027 also takes the voltage to recover at, which must be at least 0.5 V above brownout.
        RobotController.setBrownoutVoltages(Volts.of(6.1), Volts.of(6.6));
        // Swerve telemetry is published by the YAMS SwerveDrive, replacing SwerveTelemetry.
    }

    /**
     * This function is called every 20 ms, no matter the mode. The mechanism updates run first, as
     * v2 subsystem periodic() methods did, then the scheduler.
     */
    @Override
    public void robotPeriodic() {
        swerve.periodic();
        intakePivot.periodic();
        intakeRollers.periodic();
        floor.periodic();
        feeder.periodic();
        shooter.periodic();
        hood.periodic();
        hanger.periodic();

        scheduler.run();
        Telemetry.log("Scheduler", scheduler, Scheduler.proto);
    }

    @Override
    public void simulationPeriodic() {
        swerve.simulationPeriodic();
        intakePivot.simulationPeriodic();
        intakeRollers.simulationPeriodic();
        floor.simulationPeriodic();
        feeder.simulationPeriodic();
        shooter.simulationPeriodic();
        hanger.simulationPeriodic();
    }

    private Command updateVisionCommand() {
        return limelight.run(coroutine -> {
            while (true) {
                final Pose2d currentRobotPose = swerve.getPose();
                final Optional<Limelight.Measurement> measurement = limelight.getMeasurement(currentRobotPose);
                measurement.ifPresent(m -> {
                    swerve.addVisionMeasurement(
                        m.poseEstimate.pose,
                        m.poseEstimate.timestampSeconds,
                        m.standardDeviations
                    );
                });
                coroutine.yield();
            }
        })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Update Vision");
    }
}
