package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
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
 * AdvantageKit Elevator Subsystem, capable of replaying the elevator.
 */
public class ElevatorSubsystem extends SubsystemBase
{

  /**
   * AdvantageKit identifies inputs via the "Replay Bubble". Everything going to the SMC is an Output. Everything coming
   * from the SMC is an Input.
   */
  @AutoLog
  public static class ElevatorInputs
  {

    public Distance       position = Meters.of(0);
    public LinearVelocity velocity = MetersPerSecond.of(0);
    public Distance       setpoint = Meters.of(0);
    public Voltage        volts    = Volts.of(0);
    public Current        current  = Amps.of(0);

  }

  private final ElevatorInputsAutoLogged elevatorInputs = new ElevatorInputsAutoLogged();


  private final Distance         chainPitch     = Inches.of(0.25);
  private final int              toothCount     = 22;
  private final Distance         circumference  = chainPitch.times(toothCount);
  private final Distance         radius         = circumference.div(2 * Math.PI);
  private final Mass             weight         = Pounds.of(16);
  private final DCMotor          motors         = DCMotor.getNEO(1);
  private final MechanismGearing gearing        = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final SparkMax         elevatorMotor  = new SparkMax(30, SparkLowLevel.MotorType.kBrushless);
  private final SparkMax         elevatorMotor2 = new SparkMax(31, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withMechanismCircumference(circumference)
      .withClosedLoopController(new ExponentialProfilePIDController(30, 0, 0, ExponentialProfilePIDController
          .createElevatorConstraints(Volts.of(12),
                                     motors,
                                     weight,
                                     radius,
                                     gearing)))
      .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withSoftLimit(Meters.of(0), Meters.of(2))
      .withGearing(gearing)
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
      // Elevator motor2 follows Elevator motor with an inversed output.
      .withFollowers(Pair.of(elevatorMotor2, true));

  private final SmartMotorController motor      = new SparkWrapper(elevatorMotor,
                                                                   motors,
                                                                   motorConfig);
  private       ElevatorConfig       m_config   = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(weight);
  private final Elevator             m_elevator = new Elevator(m_config);

  public ElevatorSubsystem()
  {
    new Trigger(() -> getHeight().lte(Meters.of(0.1)))
        .and(() -> elevatorInputs.setpoint.isEquivalent(motorConfig.convertFromMechanism(Rotations.of(0))))
        .whileTrue(m_elevator.set(0));
  }

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
    Logger.processInputs("Elevator", elevatorInputs);
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    Logger.recordOutput("Elevator/DutyCycle", dutycycle);
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    Logger.recordOutput("Elevator/Setpoint", height);
    return m_elevator.setHeight(height);
  }

  public Command sysId()
  {
    return m_elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }

  public Distance getHeight()
  {
    return elevatorInputs.position;
  }

  public Distance getSetpoint()
  {
    return elevatorInputs.setpoint;
  }

}