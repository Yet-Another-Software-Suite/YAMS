package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorSubsystem extends SubsystemBase
{
  private final Distance chainPitch = Inches.of(0.25);
  private final int toothCount = 22;
  private final Distance circumference = chainPitch.times(toothCount);
  private final Distance radius = circumference.div(2 * Math.PI);
  private final Mass     weight = Pounds.of(16);
  private final DCMotor  motors = DCMotor.getNEO(1);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final SparkMax                    elevatorMotor      = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit(); // Specific telemetry verbosity
  private final SmartMotorControllerConfig motorConfig        = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(circumference)
//      .withFollowers(Pair.of(new SparkMax(3, SparkLowLevel.MotorType.kBrushless), true))
      .withClosedLoopController(new ExponentialProfilePIDController(30, 0, 0, ExponentialProfilePIDController
          .createElevatorConstraints(Volts.of(12),
                                     motors,
                                     weight,
                                     radius,
                                     gearing)))
//      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5)) // Trapezoidal Profile PID Controller
      .withSoftLimit(Meters.of(0), Meters.of(2))
      .withGearing(gearing)
//      .withExternalEncoder(armMotor.getAbsoluteEncoder()) // External Encoder if you need one, really shouldnt be used for Elevators
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig) // Specific Telemetry
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12)) // Voltage compensation isnt available on all controllers
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25)) // Closed Loop Ramp Rate not necessary
//      .withOpenLoopRampRate(Seconds.of(0.25)) // Open Loop Ramp Rate not necessary
      .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor              = new SparkWrapper(elevatorMotor,
                                                                                 motors,
                                                                                 motorConfig);
  private final MechanismPositionConfig    m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private       ElevatorConfig             m_config           = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(m_robotToMechanism)
      .withMass(weight);
  private final Elevator                   m_elevator         = new Elevator(m_config);

  public ElevatorSubsystem()
  {
    new Trigger(()->m_elevator.getHeight().lte(Meters.of(0.1)))
        .and(()->motor.getMechanismPositionSetpoint().orElse(Rotations.of(1)).isEquivalent(Rotations.of(0)))
        .whileTrue(m_elevator.set(0));
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
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