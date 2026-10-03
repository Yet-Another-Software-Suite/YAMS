// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.commands;

import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import first.robot.mechanisms.IndexerMechanism;
import first.robot.mechanisms.ShooterMechanism;
import first.robot.mechanisms.SwerveMechanism;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Time;

/**
 * Commands that run the shooter and indexer together: the original {@code IntakeCommand},
 * {@code OuttakeCommand}, {@code ShootAndIndexCommand} and {@code AutoShoot}. Each requires both
 * mechanisms and stops both when it ends or is canceled.
 */
public final class ShooterCommands
{

  private static final List<Data> shots = List.of(
      // TUNE HERE
      new Data(Inches.of(115), RPM.of(3250), Second.of(0.64)),
      new Data(Inches.of(135), RPM.of(3500), Second.of(0.64)),
      new Data(Inches.of(166), RPM.of(4000), Second.of(0.64)),
      new Data(Inches.of(92), RPM.of(3200), Second.of(0.64)),
      new Data(Meters.of(8.41326147155525), RPM.of(3450), Second.of(0.64)),
      new Data(Inches.of(118.25), RPM.of(3400), Second.of(.64)),
      new Data(Inches.of(74), RPM.of(2900), Second.of(.64))
  );
  @SuppressWarnings("unchecked")
  private static final InterpolatingDoubleTreeMap shotMap = InterpolatingDoubleTreeMap.ofEntries(shots.stream()
      .map(Data::toEntry)
      .toArray(Map.Entry[]::new));

  private final IndexerMechanism indexer;
  private final ShooterMechanism shooter;

  public ShooterCommands(IndexerMechanism indexer, ShooterMechanism shooter)
  {
    this.indexer = indexer;
    this.shooter = shooter;
  }

  private void stop()
  {
    shooter.setDutycycle(0);
    indexer.setDutycycle(0);
  }

  /** Run the shooter and indexer to pull fuel in until canceled. */
  public Command intake()
  {
    return Command.requiring(indexer, shooter)
                  .executing(coroutine -> {
                    while (true)
                    {
                      shooter.setDutycycle(0.4);
                      indexer.setDutycycle(0.8);
                      coroutine.yield();
                    }
                  })
                  .whenCanceled(this::stop)
                  .named("Intake");
  }

  /** Run the shooter and indexer to push fuel out until canceled. */
  public Command outtake()
  {
    return Command.requiring(indexer, shooter)
                  .executing(coroutine -> {
                    while (true)
                    {
                      shooter.setDutycycle(-0.8);
                      indexer.setDutycycle(-0.5);
                      coroutine.yield();
                    }
                  })
                  .whenCanceled(this::stop)
                  .named("Outtake");
  }

  public Command shootAndIndex(AngularVelocity goalRPM)
  {
    return shootAndIndex(() -> goalRPM, "Shoot And Index " + goalRPM.in(RPM) + " RPM");
  }

  /** Shoot with the RPM from the shot table for the robot's distance to the hub. */
  public Command shootAndIndex(SwerveMechanism swerve)
  {
    return shootAndIndex(() -> {
      return RPM.of(shotMap.get((swerve.distanceFromHubMeters().minus(Feet.of(1)).in(Meters))));
    }, "Shoot And Index From Distance");
  }

  /**
   * Spin the shooter to the goal and feed once it is within 50 RPM; otherwise hold fuel back. Runs
   * until canceled.
   */
  public Command shootAndIndex(Supplier<AngularVelocity> goal, String name)
  {
    return Command.requiring(indexer, shooter)
                  .executing(coroutine -> {
                    indexer.setDutycycle(0);
                    while (true)
                    {
                      shooter.setVelocity(goal.get());
                      Telemetry.log("Shooter/Goal", goal.get().in(RPM));
                      Telemetry.log("Shooter/RPM", shooter.getVelocity().in(RPM));
                      if ((shooter.getVelocity().isNear(goal.get(), RPM.of(50))))
                      {
                        indexer.setDutycycle(-.60);
                      } else
                      {
                        indexer.setDutycycle(0.3);
                      }
                      coroutine.yield();
                    }
                  })
                  .whenCanceled(this::stop)
                  .named(name);
  }

  /**
   * Spin up, feed once the shooter reaches 90% of the goal, and finish after feeding for the given
   * time.
   */
  public Command autoShoot(Supplier<AngularVelocity> setpoint, Time time)
  {
    return Command.requiring(indexer, shooter)
                  .executing(coroutine -> {
                    Timer   timer   = new Timer();
                    boolean feeding = false;
                    shooter.setVelocity(setpoint.get());
                    while (!(feeding && timer.get() >= time.in(Seconds)))
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
                      } else
                      {
                        indexer.setDutycycle(0);
                      }
                      coroutine.yield();
                    }
                    stop();
                  })
                  .whenCanceled(this::stop)
                  .named("Auto Shoot");
  }

  /** Stop the shooter and indexer, then end. */
  public Command stopCommand()
  {
    return Command.requiring(indexer, shooter)
                  .executing(coroutine -> stop())
                  .named("Stop");
  }

  record Data(Distance distance, AngularVelocity velocity, Time timeOfFlight)
  {

    public Map.Entry<Double, Double> toEntry()
    {
      return Map.entry(distance.in(Meters), velocity.in(RPM));
    }
  }
}
