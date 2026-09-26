// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;


import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.Feet;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ExponentialProfilePIDController;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled arm mechanism. The arm represented by this class does NOT have an absolute encoder! This
 * mechanism has a "self-homing" command, more details in the function description.
 */
public class ExponentiallyProfiledArmMechanism implements Mechanism
{
  private final String           motorTelemetryName = "ExponentiallyProfiledArmMotor";
  private final String           mechTelemetryName  = "ExponentiallyProfiledArm";
  private final SparkMax         armMotor           = new SparkMax(CANPorts.fromBusId(1), 1, MotorType.kBrushless);
  ///  Configuration Options
  private final DCMotor          dcMotor            = DCMotor.getNEO(1);
  private final MechanismGearing gearing            = new MechanismGearing(7);
  private final Mass             weight             = Pounds.of(10);
  private final Distance         length             = Feet.of(2);
  /*
   * Using the protractor, where 0deg on the protractor is when the arm is parallel to the ground,
   * you can measure where the starting angle should be.
   */
  private final Angle            startingAngle      = Degrees.of(30);
  /*
   * To find these limits measure the starting angle relative to when the arm is parallel to the ground using a protractor.
   */
  private final Angle            softLowerLimit     = Degrees.of(-20);
  private final Angle            softUpperLimit     = Degrees.of(100);
  /*
   * These are the real "limits" of the robot shown in simulation.
   */
  private final Angle            hardLowerLimit     = Degrees.of(-30);
  private final Angle            hardUpperLimit     = Degrees.of(110);

  /*
   * This is the STARTING PID Controller for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ExponentialProfilePIDController pidController  = new ExponentialProfilePIDController(1,
                                                                                                     0,
                                                                                                     0,
                                                                                                     ExponentialProfilePIDController.createArmConstraints(
                                                                                                         Volts.of(12),
                                                                                                         dcMotor,
                                                                                                         weight,
                                                                                                         length,
                                                                                                         gearing));
  /*
   * This is the STARTING Feedforward for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ArmFeedforward                  armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  /**
   * {@link SmartMotorControllerConfig} for the arm motor.
   */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
                                                /*
                                                 * Basic Configuration options for the motor
                                                 */
                                                .withMotorInverted(false)
                                                .withIdleMode(MotorMode.BRAKE)
                                                .withControlMode(ControlMode.CLOSED_LOOP)
                                                .withGearing(gearing)
                                                .withStatorCurrentLimit(Amps.of(40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
                                                .withClosedLoopRampRate(Seconds.of(0.25)) // Prevents our motor from rapid demand changes that could cause dramatic voltage drops, and current draw.
                                                .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
                                                .withTelemetry(motorTelemetryName,
                                                               TelemetryVerbosity.HIGH) // Could have more fine-grained control over what gets reported with SmartMotorControllerTelemetryConfig
                                                /*
                                                 * Closed loop configuration options for the motor.
                                                 */
                                                .withClosedLoopController(1,0,0)
          .withExponentialProfile(ExponentialProfilePIDController.createArmConstraints(
                  Volts.of(12),
                  dcMotor,
                  weight,
                  length,
                  gearing))
                                                .withFeedforward(armFeedforward)
                                                .withSoftLimits(softLowerLimit, softUpperLimit)
                                                .withStartingPosition(startingAngle); // The starting position should ONLY be defined if you are NOT using an absolute encoder.

  /// Generic Smart Motor Controller with our options and vendor motor.
  private final SmartMotorController motor    = new SparkWrapper(armMotor, dcMotor, motorConfig);
  /// Arm-specific options
  private       ArmConfig            m_config = new ArmConfig()
      /*
       * Basic configuration options for the arm.
       */
      .withLength(length)
      //.withSimStartingPosition(Degrees.of(0)) // Override the starting position for simulation only.
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the arm.
       */
      .withHardLimits(hardLowerLimit, hardUpperLimit);
  // Arm mechanism
  private final Arm                  arm      = new Arm(m_config, motor);

  public ExponentiallyProfiledArmMechanism()
  {
  }

  public void periodic()
  {
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  /**
   * Reset the encoder to the lowest position when the current threshold is reached. Should be used when the Arm
   * position is unreliable, like startup. Threshold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second. Runs above the default priority, so normal arm commands cannot interrupt it
   * before the arm is homed.
   *
   * @param threshold The current threshold held when the Arm is at its hard limit.
   * @return {@link Command} that ends once the arm is homed.
   */
  public Command homing(Current threshold)
  {
    Voltage         runVolts          = Volts.of(2); // Volts required to run the mechanism up. Could be negative if the mechanism is inverted.
    Angle           limitHit          = hardUpperLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
    AngularVelocity velocityThreshold = DegreesPerSecond.of(2); // The maximum amount of movement for the arm to be considered "hitting the hard limit".
    return run(coroutine -> {
      Debouncer currentDebouncer = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
      // The YAMS voltage command stops the closed loop controller while it runs and restarts it when it ends. As a
      // forked child it ends together with this command.
      coroutine.fork(arm.setVoltage(runVolts));
      coroutine.waitUntil(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshold) &&
                                                           motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                                                           velocityThreshold.in(DegreesPerSecond)));
      motor.setEncoderPosition(limitHit);
    }).whenCanceled(() -> motor.setEncoderPosition(limitHit))
      .withPriority(Command.DEFAULT_PRIORITY + 1)
      .named("ExponentiallyProfiledArm Homing");
  }

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

}
