// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import first.robot.mechanisms.ArmMechanism;
import first.robot.mechanisms.ElevatorMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.RobotBase;

public class Robot extends OpModeRobot {
  public final SwerveMechanism drive = new SwerveMechanism();
  public final ArmMechanism arm = new ArmMechanism();
  public final ElevatorMechanism elevator = new ElevatorMechanism();
  public final ShooterMechanism shooter = new ShooterMechanism();

  public final CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  private final Scheduler scheduler = Scheduler.getDefault();

  public Robot() {
    DriverStationBackend.silenceJoystickConnectionAlert(true);
    drive.setDefaultCommand(drive.driveWithJoystick(xboxController));
    arm.setDefaultCommand(arm.setAngle(Degrees.of(0)));
    elevator.setDefaultCommand(elevator.setHeight(Meters.of(0)));
    shooter.setDefaultCommand(shooter.setVelocity(RPM.of(0)));
  }

  @Override
  public void robotPeriodic() {
    // Mechanism periodic updates run before the scheduler, matching the commands v2 Subsystem periodic order.
    drive.periodic();
    arm.periodic();
    elevator.periodic();
    shooter.periodic();
    if (RobotBase.isSimulation()) {
      drive.simulationPeriodic();
      arm.simulationPeriodic();
      elevator.simulationPeriodic();
      shooter.simulationPeriodic();
    }

    scheduler.run();
  }
}
