// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.mechanisms;

import com.ctre.phoenix6.hardware.TalonFX;
import first.robot.Constants.CANIDS;
import first.robot.Constants.Shooter;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Pair;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.remote.TalonFXWrapper;

/** Dual Kraken X60 shooter flywheel, a YAMS {@link FlyWheel}. Also intakes and outtakes fuel. */
public class ShooterMechanism implements Mechanism
{

  private final TalonFX              motorController = new TalonFX(CANIDS.shooterCANID, CANIDS.canBus);
  // Second Kraken, inverted, follows the leader.
  private final TalonFX              followerMotor   = new TalonFX(CANIDS.shooterCANIDtwo, CANIDS.canBus);
  private final SmartMotorController shooterMotorController;
  private final FlyWheel             flyWheel;

  public ShooterMechanism()
  {
    shooterMotorController = new TalonFXWrapper(motorController,
                                                Shooter.motor,
                                                Shooter.smc.clone()
                                                           .withMechanism(this)
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

  /** Called from {@code Robot.simulationPeriodic()}. */
  public void simulationPeriodic()
  {
    flyWheel.simIterate();
  }

  /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
  public void periodic()
  {
    flyWheel.updateTelemetry();
  }

  /** Run at a duty cycle until canceled. */
  public Command setDutycycleCommand(double dutycycle)
  {
    return run(coroutine -> {
      setDutycycle(dutycycle);
      coroutine.park();
    }).named("Shooter SetDutyCycle " + dutycycle);
  }

  public Command setVelocityCommand(AngularVelocity velocity)
  {
    return setVelocityCommand(() -> velocity);
  }

  /** Hold a velocity, read every loop, until canceled. */
  public Command setVelocityCommand(Supplier<AngularVelocity> velocity)
  {
    return flyWheel.run(velocity);
  }

  public boolean isNear(AngularVelocity tolerance)
  {

    return Shooter.flyWheelRecoveryDebouncer.calculate(flyWheel.getSpeed().isNear(shooterMotorController.getMechanismSetpointVelocity().orElse(shooterMotorController.getMechanismVelocity()), tolerance));
  }
}
