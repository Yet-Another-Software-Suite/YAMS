// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Chain-driven dual-NEO elevator with AdvantageKit input logging and exponential
 * motion profiling. The second motor follows the first in inverse so a single
 * SmartMotorController drives both. Height, velocity, setpoint, volts, and
 * current are all logged through ElevatorInputs so the elevator can be replayed
 * from a log without re-energizing the mechanism.
 */
public class ElevatorSubsystem extends SubsystemBase
{
  /*
   * ElevatorInputs is the replay boundary for this subsystem. @AutoLog generates
   * ElevatorInputsAutoLogged at compile time; Logger.processInputs() uses that
   * generated class to stamp all fields with a consistent timestamp.
   *
   * Logging the setpoint alongside the position lets you see in replay whether
   * the elevator reached its goal before the next command fired.
   */
  @AutoLog
  public static class ElevatorInputs
  {
    // Carriage height derived from the motor encoder via the chain geometry.
    public Distance       position = Meters.of(0);
    public LinearVelocity velocity = MetersPerSecond.of(0);
    // Active motion-profile setpoint; stored here for replay consistency.
    public Distance       setpoint = Meters.of(0);
    // Phase voltage from the leader motor.
    public Voltage        volts    = Volts.of(0);
    public Current        current  = Amps.of(0);
  }

  private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();

  // Chain geometry: 0.25-inch pitch * 22 teeth = 5.5-inch circumference sprocket.
  // This converts motor rotations to linear carriage travel.
  private final Distance         chainPitch     = Inches.of(0.25);
  private final int              toothCount     = 22;
  private final Distance         circumference  = chainPitch.times(toothCount);
  // Effective radius used by the exponential profile constraint calculation.
  private final Distance         radius         = circumference.div(2 * Math.PI);
  private final Mass             weight         = Pounds.of(16);
  private final DCMotor          motors         = DCMotor.getNEO(1);
  // 3:4 box = 12:1 total reduction; chosen to keep carriage speed below 2 m/s
  // while giving enough torque to lift 16 lb against gravity.
  private final MechanismGearing gearing        = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  // CAN IDs 30/31 are the leader and follower NEOs.
  private final SparkMax         elevatorMotor  = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax         elevatorMotor2 = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(circumference)
      // kP=30 on a chain elevator is reasonable; the exponential profile limits
      // peak demand so high P doesn't cause integral windup near the setpoint.
      .withClosedLoopController(30, 0, 0)
      // Exponential profile constraints computed from real motor/load parameters
      // so the profile is physically achievable on the first try.
      .withProfile(ExponentialProfilePIDController
          .createElevatorConstraints(Volts.of(12),
                                     motors,
                                     weight,
                                     radius,
                                     gearing))
      // kG=0.1 V holds the elevator at any height; tune this before match play.
      // kS/kV/kA are zero until SysId is run.
      .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
      // 40 A stator limit protects the NEO during a hard-stop impact.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      // Soft limits prevent commanding the carriage outside the physical frame.
      .withSoftLimits(Meters.of(0), Meters.of(2))
      .withGearing(gearing)
      // BRAKE holds position when the command ends; avoids a gravity-driven drop.
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      // elevatorMotor2 is inverted relative to the leader because the two motors
      // face opposite directions on the carriage.
      .withFollowers(Pair.of(elevatorMotor2, true))
      // Starting height of 0.5 m prevents the sim from crashing the carriage into
      // the floor at t=0 when no homing has been performed.
      .withStartingPosition(Meters.of(0.5));

  private final SmartMotorController motor      = new SparkWrapper(elevatorMotor,
                                                                   motors,
                                                                   motorConfig);
  private       ElevatorConfig       m_config   = new ElevatorConfig()
      // Hard limits model the physical travel of the real mechanism in sim.
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withCarriageWeight(weight);

  private final Elevator m_elevator = new Elevator(m_config, motor);

  public ElevatorSubsystem()
  {
    // Safety: if the carriage is at or below 10 cm and the setpoint is floor (0 rot),
    // force duty-cycle to zero rather than letting the profile command downward
    // motion into the hard stop.
    new Trigger(() -> getHeight().lte(Meters.of(0.1)))
        .and(() -> elevatorInputs.setpoint.isEquivalent(motorConfig.convertFromMechanism(Rotations.of(0))))
        .whileTrue(m_elevator.set(0));
  }

  /**
   * Populate ElevatorInputs from the mechanism and SMC. Must be called before
   * Logger.processInputs() to guarantee all fields share the same log timestamp.
   */
  private void updateInputs()
  {
    elevatorInputs.setpoint = motorConfig.convertFromMechanism(m_elevator.getMechanismSetpoint()
                                                                         .orElse(Rotations.of(1)));
    elevatorInputs.position = m_elevator.getHeight();
    elevatorInputs.velocity = m_elevator.getVelocity();
    elevatorInputs.current = motor.getStatorCurrent();
    elevatorInputs.volts = motor.getVoltage();
  }

  public void periodic()
  {
    updateInputs();
    // processInputs stamps ElevatorInputs with the current timestamp and, during
    // replay, overwrites all fields with the recorded values from the log file.
    Logger.processInputs("Elevator", elevatorInputs);
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    // recordOutput writes duty-cycle as a computed output; it is NOT replayed.
    // This lets you audit what was commanded vs. what the elevator actually did.
    Logger.recordOutput("Elevator/DutyCycle", dutycycle);
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    // Setpoint is a command output -- recomputed during replay from the same
    // control logic, so you can verify the auto sequence still issues the same
    // height goals when replayed against modified subsystem code.
    Logger.recordOutput("Elevator/Setpoint", height);
    return m_elevator.setHeight(height);
  }

  public Distance getHeight()
  {
    // Reads from elevatorInputs so replay returns the logged sensor value, not
    // a live hardware read.
    return elevatorInputs.position;
  }

  public Distance getSetpoint()
  {
    return elevatorInputs.setpoint;
  }

}
