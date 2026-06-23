// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Open-loop belt indexer with AdvantageKit input logging. The indexer is
 * deliberately open-loop -- velocity feedback isn't needed for reliable
 * game-piece feeding, and open-loop keeps the control path simple. Logging
 * velocity, volts, and current still enables post-match diagnosis of jams
 * and brown-outs through log replay.
 */
public class IndexerSubsystem extends SubsystemBase
{
  /*
   * IndexerInputs crosses the replay boundary. @AutoLog generates
   * IndexerInputsAutoLogged; Logger.processInputs() uses it to stamp all three
   * fields with the same timestamp each loop. Because the indexer is open-loop,
   * there is no setpoint field -- the only meaningful hardware observables are
   * velocity (to detect jams), volts, and current.
   */
  @AutoLog
  public static class IndexerInputs
  {
    // Roller speed; a sharp drop while volts are applied indicates a jam.
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    public Voltage         volts    = Volts.of(0);
    public Current         current  = Amps.of(0);
  }

  private final IndexerInputsAutoLogged indexerInputs = new IndexerInputsAutoLogged();

  // CAN ID 20 -- check against the robot wiring diagram if swapping hardware.
  private final SparkMax someMotor = new SparkMax(20, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // 3:4 box = 12:1 total reduction. Fast enough for reliable feeding without
      // back-driving the rollers when the motor is released.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // COAST lets game pieces coast through after power is removed, preventing
      // double-feeds from a late motor stop.
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("IndexerMotor", TelemetryVerbosity.HIGH)
      // 40 A stator limit; jams can spike current quickly on a belt drive.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withControlMode(ControlMode.OPEN_LOOP);

  private final SmartMotorController motor = new SparkWrapper(someMotor, DCMotor.getNEO(1), motorConfig);

  /**
   * Populate IndexerInputs from the SMC. Called at the start of periodic() so
   * Logger.processInputs() can stamp the values before any consumer reads them.
   * During replay the logger overwrites these fields, making getVelocity() return
   * the recorded roller speed rather than a live hardware read.
   */
  private void updateInputs()
  {
    indexerInputs.velocity = motor.getMechanismVelocity();
    indexerInputs.volts = motor.getVoltage();
    indexerInputs.current = motor.getStatorCurrent();
  }

  public IndexerSubsystem()
  {
  }

  /**
   * Gets the current velocity of the indexer.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity()
  {
    // Reads from indexerInputs so this returns the replayed value during log
    // replay, not a live hardware read.
    return indexerInputs.velocity;
  }

  /**
   * Set the voltage of the indexer.
   *
   * @param volts Voltage to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVoltage(Voltage volts)
  {
    return run(() -> {
      // recordOutput logs the commanded voltage as a computed output; it is NOT
      // replayed. Lets you see in replay what was sent vs. what the roller did.
      Logger.recordOutput("Indexer/Voltage", volts);
      motor.setVoltage(volts);
    }).withName("IndexerSetVoltage");
  }

  /**
   * Set the dutycycle of the indexer.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle)
  {
    return run(() -> {
      Logger.recordOutput("Indexer/DutyCycle", dutyCycle);
      motor.setDutyCycle(dutyCycle);
    }).withName("IndexerSetDutyCycle");
  }

  /**
   * DutyCycle supplier controlling the indexer
   *
   * @param dutyCycle Dutycyle supplier
   * @return Command
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return run(() -> {
      Logger.recordOutput("Indexer/DutyCycle", dutyCycle.get());
      motor.setDutyCycle(dutyCycle.get());
    }).withName("IndexerSetDutyCycleSupplier");
  }

  @Override
  public void simulationPeriodic()
  {
    motor.simIterate();
  }

  @Override
  public void periodic()
  {
    updateInputs();
    // processInputs stamps IndexerInputs and closes the replay bubble.
    // After this call, getVelocity() returns the logged value in replay mode.
    Logger.processInputs("Indexer", indexerInputs);
    motor.updateTelemetry();
  }
}
