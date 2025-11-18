package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled arm subsystem. The arm represented by this class does NOT have an absolute encoder! This
 * subsystem has a "self-homing" command, more details in the function description.
 */
public class ExponentiallyProfiledArmSubsystem extends SubsystemBase
{

  private final String           motorTelemetryName = "ExponentiallyProfiledArmMotor";
  private final String           mechTelemetryName  = "ExponentiallyProfiledArm";
  private final SparkMax         armMotor           = new SparkMax(1, MotorType.kBrushless);
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
      .withClosedLoopController(pidController)
      .withFeedforward(armFeedforward)
      .withSoftLimit(softLowerLimit, softUpperLimit);

  /// Generic Smart Motor Controller with out options and vendor motor.
  private final SmartMotorController motor    = new SparkWrapper(armMotor, dcMotor, motorConfig);
  /// Arm-specific options
  private       ArmConfig            m_config = new ArmConfig(motor)
      /*
       * Basic configuration options for the arm.
       */
      .withLength(length)
      .withMass(weight)
      .withStartingPosition(startingAngle) // The starting position should ONLY be defined if you are NOT using an absolute encoder.
      //.withHorizontalZero(Degrees.of(0)) // The horizontal zero should ONLY be defined if you ARE using an absolute encoder.
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the arm.
       */
      .withHardLimit(hardLowerLimit, hardUpperLimit);
  // Arm mechanism
  private final Arm                  arm      = new Arm(m_config);

  public ExponentiallyProfiledArmSubsystem()
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
   * Reset the encoder to the lowest position when the current threshhold is reached. Should be used when the Arm
   * position is unreliable, like startup. Threshhold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second.
   *
   * @param threshhold The current threshhold held when the Arm is at it's hard limit.
   * @return
   */
  public Command homing(Current threshhold)
  {
    Debouncer       currentDebouncer  = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
    Voltage         runVolts          = Volts.of(2); // Volts required to run the mechanism up. Could be negative if the mechanism is inverted.
    Angle           limitHit          = hardUpperLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
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

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command sysId()
  {
    return arm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }
}