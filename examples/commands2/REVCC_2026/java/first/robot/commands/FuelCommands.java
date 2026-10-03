// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static org.wpilib.units.Units.RPM;

import first.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import first.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import first.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import first.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import first.robot.subsystems.ConveyorSubsystem;
import first.robot.subsystems.FeederSubsystem;
import first.robot.subsystems.IntakeSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;

/**
 * Commands that run more than one fuel mechanism together. Each mechanism is its own subsystem so
 * it can be tuned live on its own; these commands require every subsystem they drive.
 */
public final class FuelCommands
{
  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public static Command intake(IntakeSubsystem intake, ConveyorSubsystem conveyor)
  {
    return Commands.startEnd(
        () -> {
          intake.setIntakePower(IntakeSetpoints.kIntake);
          conveyor.setConveyorPower(ConveyorSetpoints.kIntake);
        }, () -> {
          intake.setIntakePower(0.0);
          conveyor.setConveyorPower(0.0);
        }, intake, conveyor).withName("Intaking");
  }

  /**
   * Command to reverse the intake motor and conveyor motors. When the command is interrupted, e.g.
   * the button is released, the motors will stop.
   */
  public static Command extake(IntakeSubsystem intake, ConveyorSubsystem conveyor)
  {
    return Commands.startEnd(
        () -> {
          intake.setIntakePower(IntakeSetpoints.kExtake);
          conveyor.setConveyorPower(ConveyorSetpoints.kExtake);
        }, () -> {
          intake.setIntakePower(0.0);
          conveyor.setConveyorPower(0.0);
        }, intake, conveyor).withName("Extaking");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public static Command feed(ShooterSubsystem shooter, FeederSubsystem feeder)
  {
    return Commands.startEnd(
        () -> {
          shooter.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
          feeder.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          shooter.setFlywheelVelocity(RPM.of(0));
          feeder.setFeederPower(0.0);
        }, shooter, feeder).withName("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches the
   * desired speed it starts the Feeder.
   */
  public static Command shoot(ShooterSubsystem shooter, FeederSubsystem feeder)
  {
    return shooter.startEnd(
        () -> shooter.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
        shooter::stopFlywheel
    ).until(shooter.isFlywheelSpinning).andThen(
        Commands.startEnd(
            () -> {
              shooter.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
              feeder.setFeederPower(FeederSetpoints.kFeed);
            }, () -> {
              shooter.stopFlywheel();
              feeder.setFeederPower(0.0);
            }, shooter, feeder)
    ).withName("Shooting");
  }

  private FuelCommands()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
