package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.DoubleJointedArm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

public class DoubleJointedArmSubsystem extends SubsystemBase
{
  private final SparkMax                   lowerMotor  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
          .withGearing(gearing(gearbox(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("LowerMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       lowerSMC    = new SparkWrapper(lowerMotor,
          DCMotor.getNEO(1),
          lowerConfig);
  private final ArmConfig        lowerArmConfig = new ArmConfig(lowerSMC)
          .withLength(Feet.of(2))
          .withHardLimit(Degrees.of(-360), Degrees.of(360))
          .withTelemetry("LowerArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withMass(Pounds.of(5))
          .withStartingPosition(Degrees.of(90));
  private final SparkMax                   upperMotor  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
          .withClosedLoopController(16, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
          .withGearing(gearing(gearbox(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("LowerMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       upperSMC    = new SparkWrapper(upperMotor,
                                                                          DCMotor.getNEO(1),
                                                                          upperConfig);
  private final ArmConfig        upperArmConfig = new ArmConfig(upperSMC)
      .withLength(Feet.of(2))
      .withHardLimit(Degrees.of(-360), Degrees.of(360))
      .withTelemetry("UpperArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withMass(Pounds.of(2))
      .withSimColor(new Color8Bit(Color.kDarkRed))
      .withStartingPosition(Degrees.of(0));
  private final DoubleJointedArm jointedArm     = new DoubleJointedArm(lowerArmConfig, upperArmConfig);

  public DoubleJointedArmSubsystem()
  {
  }

  public Command setAngle(Angle lowerAngle, Angle upperAngle) {
    return jointedArm.setAngle(lowerAngle, upperAngle);
  }

  public Command set(Double lowerDutycycle, Double upperDutycycle) {
    return jointedArm.set(lowerDutycycle, upperDutycycle);
  }

  public void periodic()
  {
    jointedArm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    jointedArm.simIterate();
  }

  public Command sysId()
  {
    return jointedArm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }
}