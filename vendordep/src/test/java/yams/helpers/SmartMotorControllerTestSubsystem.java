// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.helpers;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;

public class SmartMotorControllerTestSubsystem extends SubsystemBase
{
  public SmartMotorController smc;
  public Runnable             mechSimPeriodic     = null;
  public Runnable             mechUpdateTelemetry = null;
  public boolean              testRunning         = false;

  public SmartMotorControllerTestSubsystem()
  {
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return startRun(smc::stopClosedLoopController, () -> {smc.setDutyCycle(dutyCycle);})
        .finallyDo(smc::startClosedLoopController);
  }

  public Command setPositionSetpoint(Angle position)
  {
    return run(() -> smc.setPosition(position));
  }

  public Command setPositionSetpoint(Distance position)
  {
    return run(() -> smc.setPosition(position));
  }

  public void close()
  {
    smc.close();
  }

  public void setSMC(SmartMotorController sm)
  {
    smc = sm;
  }

  @Override
  public void periodic()
  {
    if (testRunning)
    {
      if (mechUpdateTelemetry == null)
      {
        smc.updateTelemetry();
      } else
      {
        mechUpdateTelemetry.run();
      }
    }
  }

  @Override
  public void simulationPeriodic()
  {
    if (testRunning)
    {
      if (mechSimPeriodic == null)
      {
//        System.out.println("Simulating " + smc.getName());
        smc.simIterate();
      } else
      {
        mechSimPeriodic.run();
      }
    }
  }
}
