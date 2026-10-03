// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import static org.wpilib.units.Units.RPM;

import java.util.function.Supplier;

import first.robot.subsystems.IndexerSubsystem;
import first.robot.subsystems.ShooterSubsystem;
import org.wpilib.command2.Command;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.AngularVelocity;

public class AutoShoot extends Command
{

    private ShooterSubsystem shooter;
    private IndexerSubsystem indexer;
    private Supplier<AngularVelocity> setpoint;
    private double time;
    private Timer timer = new Timer();
    private boolean feeding = false;
    boolean goingUp = true;


    public AutoShoot(Supplier<AngularVelocity> shootSpeed,
                     ShooterSubsystem shooter,
                     IndexerSubsystem indexer,
                     double time
    ){
        this.shooter = shooter;
        this.indexer = indexer;
        setpoint = shootSpeed;
        this.time = time;
        addRequirements(shooter, indexer);
    }


  /**
   * The initial subroutine of a command.  Called once when the command is initially scheduled.
   */
  @Override
  public void initialize()
  {
    shooter.setVelocity(setpoint.get());
      timer.reset();
      timer.stop();
      feeding = false;

  }

  /**
   * The main body of a command.  Called repeatedly while the command is scheduled. (That is, it is called repeatedly
   * until {@link #isFinished()}) returns true.)
   */
@Override
public void execute()
{
    shooter.setVelocity(setpoint.get());


    if (shooter.getVelocity().in(RPM) >= setpoint.get().in(RPM) * 0.90)
    {
        if (!feeding)
        {
            timer.start();   // start timing ONLY once we hit speed
            feeding = true;
        }

        indexer.setDutycycle(-1);



    }
    else
    {
        indexer.setDutycycle(0);
    }
}

  /**
   * <p>
   * Returns whether this command has finished. Once a command finishes -- indicated by this method returning true --
   * the scheduler will call its {@link #end(boolean)} method.
   * </p><p>
   * Returning false will result in the command never ending automatically. It may still be cancelled manually or
   * interrupted by another command. Hard coding this command to always return true will result in the command executing
   * once and finishing immediately. It is recommended to use *
   * {@link org.wpilib.command2.InstantCommand InstantCommand} for such an operation.
   * </p>
   *
   * @return whether this command has finished.
   */
  @Override
  public boolean isFinished()
  {
    return feeding && timer.get() >= time;
  }

  /**
   * The action to take when the command ends. Called when either the command finishes normally -- that is it is called
   * when {@link #isFinished()} returns true -- or when  it is interrupted/canceled. This is where you may want to wrap
   * up loose ends, like shutting off a motor that was being used in the command.
   *
   * @param interrupted whether the command was interrupted/canceled
   */
  @Override
  public void end(boolean interrupted)
  {


    shooter.setDutycycle(0);
    indexer.setDutycycle(0);


    }
}
