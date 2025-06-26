package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;

public class ElevatorSubsystem extends SubsystemBase
{

  private final SparkMax                   elevatorMotor = new SparkMax(2, MotorType.kBrushless);
//  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      .withSoftLimit(Meters.of(0), Meters.of(2))
      .withGearing(gearing(gearbox(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.OPEN_LOOP);
  private final SmartMotorController       motor         = new SparkWrapper(elevatorMotor,
                                                                            DCMotor.getNEO(1),
                                                                            motorConfig);
  private final ElevatorConfig             m_config      = new ElevatorConfig(motor)
      .withStartingHeight(Meters.of(0.5))
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(16));
  private final Elevator                   elevator      = new Elevator(m_config);

  public ElevatorSubsystem()
  {

  }

  public void periodic()
  {
    elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    elevator.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    return elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return elevator.setHeight(height);
  }

  public Command sysId()
  {
    return elevator.sysId(Volts.of(12), Volts.of(12).per(Second), Second.of(30));
  }
}

