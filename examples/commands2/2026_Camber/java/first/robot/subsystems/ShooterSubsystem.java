// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.CANIDS;
import first.robot.Constants.Shooter;
import java.util.function.Supplier;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Pair;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

public class ShooterSubsystem extends SubsystemBase
{

  private final TalonFX              motorController = new TalonFX(CANIDS.shooterCANID, CANIDS.canBus);
  // Second Kraken, inverted, follows the leader.
  private final TalonFX              followerMotor   = new TalonFX(CANIDS.shooterCANIDtwo, CANIDS.canBus);
  private final SmartMotorController shooterMotorController;
  private final FlyWheel             flyWheel;

  public ShooterSubsystem()
  {
    shooterMotorController = new TalonFXWrapper(motorController,
                                                Shooter.motor,
                                                Shooter.smc.clone()
                                                           .withSubsystem(this)
                                                           .withFollowers(Pair.of(followerMotor, true)));
    flyWheel = new FlyWheel(Shooter.config, shooterMotorController);
  }

  public void setDutycycle(double dutycycle)
  {
    flyWheel.setDutyCycleSetpoint(dutycycle);
  }

  public void setVelocity(AngularVelocity velocity)
  {
    flyWheel.setMechanismVelocitySetpoint(velocity);
  }

  public AngularVelocity getVelocity()
  {
    return flyWheel.getSpeed();
  }

  @Override
  public void simulationPeriodic()
  {
    flyWheel.simIterate();
  }

  @Override
  public void periodic()
  {
    flyWheel.updateTelemetry();
  }

  public Command setDutycycleCommand(double dutycycle)
  {
    return run(() -> setDutycycle(dutycycle)).withName("Shooter SetDutyCycle");
  }

  public Command setVelocityCommand(AngularVelocity velocity)
  {
    return setVelocityCommand(() -> velocity);
  }

  public Command setVelocityCommand(Supplier<AngularVelocity> velocity)
  {
    return flyWheel.run(velocity);
  }

  public boolean isNear(AngularVelocity tolerance)
  {

    return Shooter.flyWheelRecoveryDebouncer.calculate(flyWheel.getSpeed().isNear(shooterMotorController.getMechanismSetpointVelocity().orElse(shooterMotorController.getMechanismVelocity()), tolerance));
  }
}
