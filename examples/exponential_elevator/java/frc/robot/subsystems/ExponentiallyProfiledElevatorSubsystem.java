// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled elevator subsystem. The elevator represented by this class does NOT have an absolute encoder! This
 * subsystem has a "self-homing" command, more details in the function description.
 *
 * <p>This example uses an exponential motion profile instead of a trapezoidal one. The exponential
 * profile reaches the target with a smoothly decaying velocity curve rather than a flat cruise
 * phase, which reduces mechanical shock on the chain and sprocket at the cost of slightly longer
 * travel time at low speeds.</p>
 */
public class ExponentiallyProfiledElevatorSubsystem extends SubsystemBase
{
  private final String           motorTelemetryName = "ExponentiallyProfiledElevatorMotor";
  private final String           mechTelemetryName  = "ExponentiallyProfiledElevator";
  // CAN ID 2 -- change to match your robot's CAN bus assignment.
  private final SparkMax         elevatorMotor      = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  ///  Configuration Options
  // NEO rated at 5880 RPM free speed and 2.6 N*m stall torque; matches most FRC elevator designs.
  private final DCMotor          dcMotor            = DCMotor.getNEO(1);
  /*
   * #25 roller chain has a 0.25-inch pitch; this value converts sprocket teeth to arc length.
   * If you switch to #35 chain, change this to Inches.of(0.375).
   */
  private final Distance         chainPitch         = Inches.of(0.25);
  /*
   * A 22-tooth #25 sprocket gives a circumference of 5.5 inches (~139.7 mm).
   * Larger tooth counts increase top speed but reduce pushing force for the same motor.
   */
  private final int              toothCount         = 22;
  // Derived from pitch * teeth; used as the mechanism circumference so motor rotations map to meters.
  private final Distance         circumference      = chainPitch.times(toothCount);
  // Radius is only needed for the exponential profile constraint calculation (torque arm length).
  private final Distance         radius             = circumference.div(2 * Math.PI);
  /*
   * 3:1 and 4:1 stages in series give a 12:1 reduction total.
   * Higher reduction increases force but lowers maximum carriage speed.
   */
  private final MechanismGearing gearing            = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  /*
   * 16 lb is a typical FRC carriage weight including game piece manipulator.
   * This value feeds the exponential profile constraint so the motor voltage
   * budget accounts for the gravitational load correctly.
   */
  private final Mass             weight             = Pounds.of(16);
  /*
   * Using a measuring tape, where 0 m marks the elevator at its lowest point,
   * you can measure the height to determine the starting position reference.
   * This value is 0 because the elevator rests at the bottom at robot power-on.
   */
  private final Distance            startingHeight      = Meters.of(0);
  /*
   * Soft limits protect the mechanism in software before hardware limit switches trigger.
   * Lower is 0 m (fully retracted); upper is 2 m, leaving 1 m of margin before the hard stop.
   * Adjust these to match your physical robot after measuring travel with a tape measure.
   */
  private final Distance            softLowerLimit     = Meters.of(0);
  private final Distance            softUpperLimit     = Meters.of(2);
  /*
   * Hard limits define the physical stops modeled in simulation.
   * The extra 1 m beyond the soft upper limit (3 m vs. 2 m) represents the
   * physical frame that the carriage would strike if software limits failed.
   */
  private final Distance            hardLowerLimit     = Meters.of(0);
  private final Distance            hardUpperLimit     = Meters.of(3);
  /*
   * All feedforward gains start at 0. Run the mechanism and use SysId (WPILib system identification)
   * to measure ks (static friction), kg (gravity compensation), kv (velocity gain), and ka (acceleration gain).
   * A non-zero kg is especially important for vertical elevators to hold position against gravity.
   */
  private final ElevatorFeedforward             elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
  /**
  * {@link SmartMotorControllerConfig} for the elevator motor.
  */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
      /*
       * Basic Configuration options for the motor
       */
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE) // BRAKE holds position better than COAST on a vertical elevator.
      .withControlMode(ControlMode.CLOSED_LOOP)
      // circumference converts motor shaft rotations into linear carriage displacement (meters).
      .withMechanismCircumference(circumference)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withStatorCurrentLimit(Amps.of(40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
      .withClosedLoopRampRate(Seconds.of(0.25)) // Prevents our motor from rapid demand changes that could cause dramatic voltage drops, and current draw.
      .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
      .withTelemetry(motorTelemetryName,
                     TelemetryVerbosity.HIGH) // Could have more fine-grained control over what gets reported with SmartMotorControllerTelemetryConfig
      /*
       * Closed loop configuration options for the motor.
       * withExponentialProfile builds the motion constraints from first principles: given 12 V max,
       * the NEO's torque curve, a 16 lb load, and the sprocket radius, the library calculates the
       * maximum achievable acceleration at every velocity point along the profile.
       */
      .withExponentialProfile(Volts.of(12), dcMotor, weight, radius)
      // P=1 is a starting point; increase until oscillation appears, then back off.
      // I and D are left at 0 because the feedforward handles steady-state error on a well-tuned system.
      .withClosedLoopController(1, 0, 0)
      .withFeedforward(elevatorFeedforward)
      .withSoftLimits(softLowerLimit, softUpperLimit)
      .withStartingPosition(startingHeight); // Starting position must live here, not on ElevatorConfig.
  /// Generic Smart Motor Controller with our options and vendor motor.
  private final SmartMotorController motor         = new SparkWrapper(elevatorMotor, dcMotor, motorConfig);
  /// Elevator-specific options
  private       ElevatorConfig       m_config      = new ElevatorConfig()
      /*
       * Basic configuration options for the elevator.
       */
      .withCarriageWeight(weight) // 16 lb carriage; used by the sim physics to compute gravitational load.
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the elevator.
       * Hard limits define the physical walls the sim carriage would hit; they do not affect real hardware.
       */
      .withHardLimits(hardLowerLimit, hardUpperLimit);
  // Elevator mechanism
  private final Elevator             m_elevator    = new Elevator(m_config, motor);

  public ExponentiallyProfiledElevatorSubsystem()
  {
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  /**
   * Drive the elevator toward its lower hard stop at -2 V until the stator current exceeds the
   * given threshold for at least 0.4 seconds while velocity stays under 2 deg/s. At that point the
   * carriage is assumed to be resting against the hard stop, so the encoder is zeroed to
   * {@code hardUpperLimit} and closed-loop control is re-enabled.
   *
   * <p>Run this command once at teleop init when the robot has been sitting powered off and the
   * encoder position is unknown. Do not run it mid-match or while the elevator is loaded.</p>
   *
   * @param threshold Stator current above which the carriage is considered stalled against its stop.
   *                  Start around 10-15 A and tune down until false triggers disappear.
   * @return Command that homes the elevator and exits automatically.
   */
  public Command homing(Current threshold)
  {
      // Current must stay above threshold for 0.4 s to rule out brief spikes during motion start.
      Debouncer       currentDebouncer  = new Debouncer(0.4);
      // Negative voltage drives the carriage downward; magnitude is low to avoid slamming the stop.
      Voltage         runVolts          = Volts.of(-2);
      // After homing, the encoder is set to the lower hard stop position (0 m).
      Distance        limitHit          = hardLowerLimit;
      // Velocity guard: if the carriage is still moving faster than this, it has not truly stalled.
      AngularVelocity velocityThreshold = DegreesPerSecond.of(2);
      return Commands.startRun(motor::stopClosedLoopController, // Stop the closed loop controller
                      () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
              .until(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshold) &&
                      motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                              velocityThreshold.in(DegreesPerSecond)))
              .finallyDo(() -> {
                  motor.setEncoderPosition(limitHit);
                  motor.startClosedLoopController();
              });
  }

  /**
   * Open-loop duty cycle command for manual override or testing without a profile.
   * Positive duty cycle moves the carriage up; negative moves it down.
   *
   * @param dutycycle Duty cycle in [-1, 1].
   * @return Command that runs until interrupted.
   */
  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  /**
   * Profile the elevator to the given height and hold it there.
   * The exponential profile drives the motor; the PID + feedforward correct for any remaining error.
   *
   * @param height Target height in meters. Must be within the soft limits (0 m to 2 m).
   * @return Command that runs until interrupted.
   */
  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

}
