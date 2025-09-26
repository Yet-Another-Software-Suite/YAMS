package yams.helpers;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorController;

public class SmartMotorControllerTestSubsystem extends SubsystemBase {
    public SmartMotorController smc;
    public Runnable mechSimPeriodic = null;
    public Runnable mechUpdateTelemetry = null;
  public boolean testRunning = false;

    public SmartMotorControllerTestSubsystem() {
    }

    public Command setDutyCycle(double dutyCycle)
    {
        return runOnce(smc::stopClosedLoopController)
                .andThen(run(()->{smc.setDutyCycle(dutyCycle);}))
                .finallyDo(smc::startClosedLoopController);
    }

    public Command setPositionSetpoint(Angle position)
    {
        return run(()->smc.setPosition(position));
    }

    public Command setPositionSetpoint(Distance position)
    {
        return run(()->smc.setPosition(position));
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
    public void periodic() {
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
    public void simulationPeriodic() {
      if (testRunning)
      {
        if (mechSimPeriodic == null)
        {
          smc.simIterate();
        } else
        {
          mechSimPeriodic.run();
        }
      }
    }
}
