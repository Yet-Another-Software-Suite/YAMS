package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.mechanisms.config.ShooterConfig;
import yams.mechanisms.velocity.Shooter;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ShooterSubsystem extends SubsystemBase
{
  private final SparkMax                   armMotor    = new SparkMax(1, MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimit(Degrees.of(-30), Degrees.of(100))
      .withGearing(gearing(gearbox(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor       = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);
  private final ShooterConfig shooterConfig = new ShooterConfig(motor)
      .withDiameter(Inches.of(4))
      .withMass(Pounds.of(1))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withUpperSoftLimit(RPM.of(1000));
  private final Shooter       shooter       = new Shooter(shooterConfig);

  public ShooterSubsystem() {}

  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  public Command setDutyCycle(double dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.setSpeed(speed);}

  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}
}