// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.CANIDS;
import first.robot.Constants.Indexer;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.local.SparkWrapper;

public class IndexerSubsystem extends SubsystemBase
{

  private SparkMax spark = new SparkMax(CANIDS.canPort, CANIDS.indexerCANID, MotorType.kBrushless);

  private SmartMotorController indexerSMC = new SparkWrapper(spark,
                                                             Indexer.motor,
                                                             Indexer.smc.clone().withSubsystem(this));

  private FlyWheel indexer = new FlyWheel(Indexer.config.clone(), indexerSMC);

  public void setDutycycle(double dutyCycle)
  {
    indexer.setDutyCycleSetpoint(dutyCycle);
  }

  public void setVelocity(AngularVelocity velocity)
  {
    indexer.setMechanismVelocitySetpoint(velocity);
  }

  @Override
  public void periodic()
  {
    // This method will be called once per scheduler run
    indexer.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    // This method will be called once per scheduler run during simulation
    indexer.simIterate();
  }

  public Command setDutycycleCommand(double dutycycle)
  {
    return run(() -> setDutycycle(dutycycle)).withName("Indexer SetDutyCycle");
  }

  public void setDutyCycleCommand(double dutycycle)
  {
    indexer.setDutyCycleSetpoint(dutycycle);
  }
}
