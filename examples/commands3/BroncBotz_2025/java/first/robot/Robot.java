// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from BroncBotz 3481 FRC2025 (comp branch).

package first.robot;

import static org.wpilib.units.Units.Seconds;

import first.robot.Constants.AlgaeArmConstants;
import first.robot.Constants.CoralArmConstants;
import first.robot.Constants.ElevatorConstants;
import first.robot.Constants.OperatorConstants;
import first.robot.commands.Drive;
import first.robot.commands.SuperstructureCommands;
import first.robot.mechanisms.AlgaeArm;
import first.robot.mechanisms.AlgaeIntake;
import first.robot.mechanisms.CoralArm;
import first.robot.mechanisms.CoralIntake;
import first.robot.mechanisms.Elevator;
import first.robot.mechanisms.Swerve;
import first.robot.util.Launchpad;
import first.robot.util.ReefTargeting;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.Trigger;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.util.Color;

/**
 * Holds the robot's mechanisms, controllers, and the commands shared by the opmodes in
 * {@code first.robot.opmodes}. OpModeRobot finds the {@code @Teleop} and {@code @Autonomous}
 * opmodes in that package and constructs the one selected on the driver station.
 */
public class Robot extends OpModeRobot {
    public final Swerve swerve = new Swerve();
    public final Elevator elevator = new Elevator();
    public final CoralArm coralArm = new CoralArm();
    public final CoralIntake coralIntake = new CoralIntake();
    public final AlgaeArm algaeArm = new AlgaeArm();
    public final AlgaeIntake algaeIntake = new AlgaeIntake();
    public final ReefTargeting targeting = new ReefTargeting();

    public final CommandNiDsXboxController driver = new CommandNiDsXboxController(OperatorConstants.kDriverControllerPort);
    public final CommandNiDsXboxController operator = new CommandNiDsXboxController(OperatorConstants.kOperatorControllerPort);
    public final Launchpad launchpad = new Launchpad(OperatorConstants.kLaunchpadPort1, OperatorConstants.kLaunchpadPort2,
        OperatorConstants.kLaunchpadPort3, Launchpad.kPressed);

    /** Commands that use several mechanisms, shared by teleop and autonomous. */
    public final SuperstructureCommands superstructure =
        new SuperstructureCommands(swerve, elevator, coralArm, coralIntake, algaeArm, algaeIntake, targeting);

    private final Scheduler scheduler = Scheduler.getDefault();

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code. Default commands set here apply in every opmode.
     */
    public Robot() {
        swerve.setDefaultCommand(Drive.teleop(swerve, driver));
        // Return the elevator to the bottom and hold the arms where they are.
        elevator.setDefaultCommand(elevator.holdAt(ElevatorConstants.kMinHeight));
        coralArm.setDefaultCommand(coralArm.holdCurrent());
        algaeArm.setDefaultCommand(algaeArm.holdCurrent());
        // Gently hold any game piece; the algae only while its arm is raised.
        coralIntake.setDefaultCommand(coralIntake.hold(coralArm::isCoralLoaded));
        algaeIntake.setDefaultCommand(algaeIntake.hold(
            () -> algaeArm.isAlgaeLoaded() && algaeArm.getAngle().gte(AlgaeArmConstants.kHoldAboveAngle)));

        // Loaded indicators on the Launchpad, in every mode.
        launchpad.setColor(0, 8, Launchpad.kUnloaded);
        launchpad.setColor(0, 7, Launchpad.kUnloaded);
        final Trigger coralLoaded = new Trigger(coralArm::isCoralLoaded);
        coralLoaded.onTrue(setPadColor(0, 8, Launchpad.kCoralLoaded));
        coralLoaded.onFalse(setPadColor(0, 8, Launchpad.kUnloaded));
        final Trigger algaeLoaded = new Trigger(algaeArm::isAlgaeLoaded);
        algaeLoaded.onTrue(setPadColor(0, 7, Launchpad.kAlgaeLoaded));
        algaeLoaded.onFalse(setPadColor(0, 7, Launchpad.kUnloaded));

        if (RobotBase.isSimulation()) {
            configureSimulatedGamePieces();
        }
    }

    private Command setPadColor(int x, int y, Color color) {
        return Command.noRequirements(coroutine -> launchpad.setColor(x, y, color))
            .named("Launchpad (" + x + ", " + y + ") " + color);
    }

    /**
     * The simulator has no game pieces, so load a coral after half a second of intaking at the human
     * player station and an algae after half a second of intaking, and drop them when spat out.
     */
    private void configureSimulatedGamePieces() {
        new Trigger(() -> coralIntake.getRollerDutyCycle() > 0.3 && coralArm.isNear(CoralArmConstants.HP))
            .debounce(Seconds.of(0.5))
            .onTrue(Command.noRequirements(coroutine -> coralArm.setSimCoralLoaded(true)).named("Sim Load Coral"));
        new Trigger(() -> coralIntake.getRollerDutyCycle() < -0.3)
            .debounce(Seconds.of(0.25))
            .onTrue(Command.noRequirements(coroutine -> coralArm.setSimCoralLoaded(false)).named("Sim Drop Coral"));
        new Trigger(() -> algaeIntake.getDutyCycle() > 0.5)
            .debounce(Seconds.of(0.5))
            .onTrue(Command.noRequirements(coroutine -> algaeArm.setSimAlgaeLoaded(true)).named("Sim Load Algae"));
        new Trigger(() -> algaeIntake.getDutyCycle() < -0.5)
            .debounce(Seconds.of(0.25))
            .onTrue(Command.noRequirements(coroutine -> algaeArm.setSimAlgaeLoaded(false)).named("Sim Drop Algae"));
    }

    /**
     * This function is called every 20 ms, no matter the mode. The mechanism updates run first, as
     * v2 subsystem periodic() methods did, then the scheduler.
     */
    @Override
    public void robotPeriodic() {
        swerve.periodic();
        elevator.periodic();
        coralArm.periodic();
        coralIntake.periodic();
        algaeArm.periodic();
        algaeIntake.periodic();

        scheduler.run();
        Telemetry.log("Scheduler", scheduler, Scheduler.proto);
    }

    @Override
    public void simulationPeriodic() {
        swerve.simulationPeriodic();
        elevator.simulationPeriodic();
        coralArm.simulationPeriodic();
        coralIntake.simulationPeriodic();
        algaeArm.simulationPeriodic();
        algaeIntake.simulationPeriodic();
    }

    /**
     * Simulation starts with an unknown alliance, so the reef poses would not be mirrored. Default to
     * Blue 1; another alliance can still be picked in the simulated driver station.
     */
    @Override
    public void simulationInit() {
        if (DriverStationSim.getAllianceStationId() == AllianceStationID.UNKNOWN) {
            DriverStationSim.setAllianceStationId(AllianceStationID.BLUE_1);
            DriverStationSim.notifyNewData();
        }
    }
}
