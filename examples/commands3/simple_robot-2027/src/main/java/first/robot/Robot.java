// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.ArmMechanism;
import first.robot.mechanisms.ElevatorMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.command3.button.CommandNiDsXboxController;
import org.wpilib.driverstation.GenericHID.RumbleType;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.OpModeRobot;
import org.wpilib.framework.RobotBase;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

public class Robot extends OpModeRobot {
  private static final Angle SCORE_ANGLE = Degrees.of(80);
  private static final Distance SCORE_HEIGHT = Meters.of(1.5);
  private static final AngularVelocity SCORE_VELOCITY = RPM.of(3500);
  private static final Time SPIN_UP_TIMEOUT = Seconds.of(2);
  private static final Time SHOT_TIME = Seconds.of(0.5);

  public final SwerveMechanism drive = new SwerveMechanism();
  public final ArmMechanism arm = new ArmMechanism();
  public final ElevatorMechanism elevator = new ElevatorMechanism();
  public final ShooterMechanism shooter = new ShooterMechanism();

  public final CommandNiDsXboxController xboxController = new CommandNiDsXboxController(0);

  private final Scheduler scheduler = Scheduler.getDefault();

  public Robot() {
    DriverStationBackend.silenceJoystickConnectionAlert(true);
    // Global defaults, at the lowest priority. The swerve drive default is set by the teleop opmode, so autos never
    // read the sticks.
    arm.setDefaultCommand(arm.stow());
    elevator.setDefaultCommand(elevator.stow());
    shooter.setDefaultCommand(shooter.stop());
  }

  /**
   * Raise the elevator and arm, spin up the shooter, and shoot. This command requires no mechanisms itself; each
   * mechanism is only owned while one of its commands runs, so the stow defaults take back over when it ends.
   *
   * @return {@link Command} that scores and then ends.
   */
  public Command scoreHigh() {
    return Command.noRequirements(coroutine -> {
      // Command-scoped default commands: after each move below ends, the mechanism holds the scoring setpoint
      // instead of stowing, and the shooter starts spinning up right away. They are reset to the global defaults
      // when this command ends.
      arm.setDefaultCommand(arm.hold(SCORE_ANGLE));
      elevator.setDefaultCommand(elevator.hold(SCORE_HEIGHT));
      shooter.setDefaultCommand(shooter.hold(SCORE_VELOCITY));

      // Command-scoped trigger: rumble the controller while the shooter is at speed. The binding is removed
      // when this command ends.
      shooter.atSpeed(SCORE_VELOCITY).whileTrue(rumble());

      coroutine.awaitAll(elevator.moveTo(SCORE_HEIGHT), arm.moveTo(SCORE_ANGLE));

      // Shoot anyway if the shooter does not reach speed in time.
      coroutine.waitUntil(shooter.atSpeed(SCORE_VELOCITY), SPIN_UP_TIMEOUT);
      coroutine.wait(SHOT_TIME);
    }).named("Score High");
  }

  /**
   * Rumble the driver controller until canceled.
   *
   * @return {@link Command} that rumbles the controller.
   */
  private Command rumble() {
    var hid = xboxController.getNiDsXboxController();
    return Command.noRequirements(coroutine -> {
      hid.setRumble(RumbleType.LEFT_RUMBLE, 0.5);
      hid.setRumble(RumbleType.RIGHT_RUMBLE, 0.5);
      coroutine.park();
    }).whenCanceled(() -> {
      hid.setRumble(RumbleType.LEFT_RUMBLE, 0);
      hid.setRumble(RumbleType.RIGHT_RUMBLE, 0);
    }).named("Rumble");
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
