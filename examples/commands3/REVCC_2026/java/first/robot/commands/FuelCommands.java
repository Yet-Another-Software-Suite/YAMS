// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.commands;

import static org.wpilib.units.Units.RPM;

import first.robot.Constants.IntakeSubsystemConstants.ConveyorSetpoints;
import first.robot.Constants.IntakeSubsystemConstants.IntakeSetpoints;
import first.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import first.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import first.robot.mechanisms.ConveyorMechanism;
import first.robot.mechanisms.FeederMechanism;
import first.robot.mechanisms.IntakeMechanism;
import first.robot.mechanisms.ShooterMechanism;
import org.wpilib.command3.Command;

/**
 * Commands that run more than one fuel mechanism together. Each mechanism is its own
 * {@link org.wpilib.command3.Mechanism} so it can be tuned live on its own; these coroutines
 * require every mechanism they drive and set each mechanism's outputs directly.
 */
public final class FuelCommands
{
  /**
   * Command to run the intake and conveyor motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public static Command intake(IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.requiring(intake, conveyor).executing(coroutine -> {
      intake.setPower(IntakeSetpoints.kIntake);
      conveyor.setPower(ConveyorSetpoints.kIntake);
      coroutine.park();
    }).whenCanceled(() -> {
      intake.stop();
      conveyor.stop();
    }).named("Intaking");
  }

  /**
   * Command to reverse the intake motor and conveyor motors. When the command is interrupted, e.g.
   * the button is released, the motors will stop.
   */
  public static Command extake(IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.requiring(intake, conveyor).executing(coroutine -> {
      intake.setPower(IntakeSetpoints.kExtake);
      conveyor.setPower(ConveyorSetpoints.kExtake);
      coroutine.park();
    }).whenCanceled(() -> {
      intake.stop();
      conveyor.stop();
    }).named("Extaking");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the feeder stops and the flywheel is driven to 0 RPM.
   */
  public static Command feed(ShooterMechanism shooter, FeederMechanism feeder)
  {
    return Command.requiring(shooter, feeder).executing(coroutine -> {
      shooter.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
      feeder.setPower(FeederSetpoints.kFeed);
      coroutine.park();
    }).whenCanceled(() -> {
      shooter.setFlywheelVelocity(RPM.of(0));
      feeder.stop();
    }).named("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches the
   * desired speed it starts the Feeder. When the command is interrupted the feeder stops and the
   * flywheel coasts down.
   */
  public static Command shoot(ShooterMechanism shooter, FeederMechanism feeder)
  {
    return Command.requiring(shooter, feeder).executing(coroutine -> {
      shooter.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
      coroutine.waitUntil(shooter.isFlywheelSpinning);
      feeder.setPower(FeederSetpoints.kFeed);
      coroutine.park();
    }).whenCanceled(() -> {
      shooter.stopFlywheel();
      feeder.stop();
    }).named("Shooting");
  }

  /** Runs {@link #shoot} and {@link #intake} together until canceled. */
  public static Command shootAndIntake(ShooterMechanism shooter, FeederMechanism feeder,
                                       IntakeMechanism intake, ConveyorMechanism conveyor)
  {
    return Command.noRequirements(coroutine -> {
      coroutine.awaitAll(shoot(shooter, feeder), intake(intake, conveyor));
    }).named("Shooting and Intaking");
  }

  private FuelCommands()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
