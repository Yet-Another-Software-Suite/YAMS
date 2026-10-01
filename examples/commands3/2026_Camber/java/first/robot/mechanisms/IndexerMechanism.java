// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.mechanisms;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.CANIDS;
import first.robot.Constants.Indexer;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;

/** NEO belt/roller stage that feeds fuel into the shooter, a YAMS {@link FlyWheel}. */
public class IndexerMechanism implements Mechanism
{

  private SparkMax spark = new SparkMax(CANIDS.canPort, CANIDS.indexerCANID, MotorType.kBrushless);

  private SmartMotorController indexerSMC = new SparkWrapper(spark,
                                                             Indexer.motor,
                                                             Indexer.smc.clone().withMechanism(this));

  private FlyWheel indexer = new FlyWheel(Indexer.config.clone(), indexerSMC);

  public void setDutycycle(double dutyCycle)
  {
    indexer.setDutyCycleSetpoint(dutyCycle);
  }

  public void setVelocity(AngularVelocity velocity)
  {
    indexer.setMechanismVelocitySetpoint(velocity);
  }

  /** Called from {@code Robot.robotPeriodic()}, replacing the v2 subsystem periodic. */
  public void periodic()
  {
    indexer.updateTelemetry();
  }

  /** Called from {@code Robot.simulationPeriodic()}. */
  public void simulationPeriodic()
  {
    indexer.simIterate();
  }

  /** Run at a duty cycle until canceled. */
  public Command setDutycycleCommand(double dutycycle)
  {
    return run(coroutine -> {
      setDutycycle(dutycycle);
      coroutine.park();
    }).named("Indexer SetDutyCycle " + dutycycle);
  }

  /** Stop the indexer and keep it stopped. The default command, at the lowest priority. */
  @Override
  public Command idle()
  {
    return run(coroutine -> {
      setDutycycle(0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY).named("Indexer Idle");
  }
}
