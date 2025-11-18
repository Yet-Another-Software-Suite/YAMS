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
import yams.math.ExponentialProfilePIDController;
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
 */
public class ExponentiallyProfiledElevatorSubsystem extends SubsystemBase
{
  private final String           motorTelemetryName = "ExponentiallyProfiledElevatorMotor";
  private final String           mechTelemetryName  = "ExponentiallyProfiledElevator";
  private final SparkMax         elevatorMotor      = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  ///  Configuration Options
  private final DCMotor          dcMotor            = DCMotor.getNEO(1);
  private final Distance         chainPitch         = Inches.of(0.25);
  private final int              toothCount         = 22;
  private final Distance         circumference      = chainPitch.times(toothCount);
  private final Distance         radius             = circumference.div(2 * Math.PI);
  private final MechanismGearing gearing            = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final Mass             weight             = Pounds.of(16);
  /*
   * Using a measuring tape, where 0 cm marks the elevator at its lowest point,
   * you can measure the height to determine the starting position reference.
   */
  private final Distance            startingHeight      = Meters.of(0);
  /*
   * To find these limits, measure the starting height relative to the elevator's lowest position using a measuring tape or ruler.
   */
  private final Distance            softLowerLimit     = Meters.of(0);
  private final Distance            softUpperLimit     = Meters.of(2);
  /*
   * These are the real "limits" of the robot shown in simulation.
   */
  private final Distance            hardLowerLimit     = Meters.of(0);
  private final Distance            hardUpperLimit     = Meters.of(3);
  /*
   * This is the STARTING PID Controller for the Elevator. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   */
  private final ExponentialProfilePIDController pidController  = new ExponentialProfilePIDController(1,
                                                                                                     0,
                                                                                                     0,
                                                                                                     ExponentialProfilePIDController.createElevatorConstraints(
                                                                                                           Volts.of(12),
                                                                                                           dcMotor,
                                                                                                           weight,
                                                                                                           radius,
                                                                                                           gearing));
  /*
   * This is the STARTING Feedforward for the Elevator. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
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
      .withIdleMode(MotorMode.BRAKE)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(circumference)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withStatorCurrentLimit(Amps.of(40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
      .withClosedLoopRampRate(Seconds.of(0.25)) // Prevents our motor from rapid demand changes that could cause dramatic voltage drops, and current draw.
      .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
      .withTelemetry(motorTelemetryName,
                     TelemetryVerbosity.HIGH) // Could have more fine-grained control over what gets reported with SmartMotorControllerTelemetryConfig
      /*
       * Closed loop configuration options for the motor.
       */
      .withClosedLoopController(pidController)
      .withFeedforward(elevatorFeedforward)
      .withSoftLimit(softLowerLimit, softUpperLimit);
  /// Generic Smart Motor Controller with our options and vendor motor.
  private final SmartMotorController motor         = new SparkWrapper(elevatorMotor, dcMotor, motorConfig);
  /// Elevator-specific options
  private       ElevatorConfig       m_config      = new ElevatorConfig(motor)
      /*
       * Basic configuration options for the arm.
       */
      .withMass(weight)
      .withStartingHeight(startingHeight) // The starting position should ONLY be defined if you are NOT using an absolute encoder.
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the arm.
       */
      .withHardLimits(hardLowerLimit, hardUpperLimit);
  // Arm mechanism
  private final Elevator             m_elevator    = new Elevator(m_config);

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
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used when the Elevator
   * position is unreliable, like startup. Threshhold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Elevator is at it's hard limit.
   * @return
   */
  public Command homing(Current threshhold)
  {
      Debouncer       currentDebouncer  = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
      Voltage runVolts          = Volts.of(-2); // Volts required to run the mechanism down. Could be negative if the mechanism is inverted.
      Distance limitHit          = hardUpperLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
      AngularVelocity velocityThreshold = DegreesPerSecond.of(2); // The maximum amount of movement for the arm to be considered "hitting the hard limit".
      return Commands.startRun(motor::stopClosedLoopController, // Stop the closed loop controller
                      () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
              .until(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshhold) &&
                      motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                              velocityThreshold.in(DegreesPerSecond)))
              .finallyDo(() -> {
                  motor.setEncoderPosition(limitHit);
                  motor.startClosedLoopController();
              });
  }

  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

  public Command sysId()
  {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }
}

