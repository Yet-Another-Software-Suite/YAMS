// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.subsystems.ADRCArmSubsystem;
import first.robot.subsystems.ADRCElevatorSubsystem;
import first.robot.subsystems.ADRCShooterSubsystem;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.button.CommandNiDsXboxController;
import org.wpilib.framework.TimedRobot;

/**
 * An arm, an elevator and a shooter run by ADRCs, written by FRC Team 8331. If you change the name of this class or
 * the package after creating this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  public final ADRCArmSubsystem arm = new ADRCArmSubsystem();
  public final ADRCElevatorSubsystem elevator = new ADRCElevatorSubsystem();
  public final ADRCShooterSubsystem shooter = new ADRCShooterSubsystem();

  private final CommandNiDsXboxController operatorController = new CommandNiDsXboxController(Constants.OPERATOR_CONTROLLER_PORT);

  public Robot() {
    // A raises the arm level, B stows it.
    operatorController.a().onTrue(arm.setAngle(Degrees.of(0)));
    operatorController.b().onTrue(arm.setAngle(Degrees.of(-20)));
    // Y raises the elevator to 1 m, X lowers it.
    operatorController.y().onTrue(elevator.setHeight(Meters.of(1)));
    operatorController.x().onTrue(elevator.setHeight(Meters.of(0)));
    // Hold the right bumper to spin the shooter up.
    operatorController.rightBumper().whileTrue(shooter.spinUp(RPM.of(3000)));
  }

  @Override
  public void robotPeriodic() {
    // Runs the scheduler, and with it each subsystem's periodic(), which publishes the YAMS telemetry.
    CommandScheduler.getInstance().run();
  }
}
