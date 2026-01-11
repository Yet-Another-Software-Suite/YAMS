package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class DiffDriveSubsystem extends SubsystemBase
{

  private MechanismGearing gearing = new MechanismGearing(3, 4);
  private Distance         wheelDiameter = Inches.of(4);

  private SparkMax leftMotor  = new SparkMax(21, SparkMax.MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(24, SparkMax.MotorType.kBrushless);

  private SparkMax leftFollowerMotor  = new SparkMax(22, SparkMax.MotorType.kBrushless);
  private SparkMax rightFollowerMotor = new SparkMax(23, SparkMax.MotorType.kBrushless);

  private SmartMotorControllerConfig leftMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(gearing)
      .withIdleMode(MotorMode.COAST)
      .withMotorInverted(true)
      .withWheelDiameter(wheelDiameter)
      .withTelemetry("LeftMotorMain", TelemetryVerbosity.LOW)
      .withFollowers(Pair.of(leftFollowerMotor, false));

  private SmartMotorControllerConfig rightMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(gearing)
      .withIdleMode(MotorMode.COAST)
      .withMotorInverted(false)
      .withWheelDiameter(wheelDiameter)
      .withTelemetry("RightMotorMain", TelemetryVerbosity.LOW)
      .withFollowers(Pair.of(rightFollowerMotor, false));

  private SmartMotorController leftMotorController  = new SparkWrapper(leftMotor, DCMotor.getNEO(2), leftMotorConfig);
  private SmartMotorController rightMotorController = new SparkWrapper(rightMotor, DCMotor.getNEO(2), rightMotorConfig);

  private DifferentialDrive drive = new DifferentialDrive(leftMotorController::setDutyCycle,
                                                          rightMotorController::setDutyCycle);

  public DiffDriveSubsystem()
  {
    setDefaultCommand(stop());
  }

  public Command stop()
  {
    return run(drive::stopMotor);
  }

  public Command tankDrive(DoubleSupplier left, DoubleSupplier right)
  {
    return run(() -> drive.tankDrive(left.getAsDouble(), right.getAsDouble()));
  }

  public Command arcadeDrive(DoubleSupplier xSpeed, DoubleSupplier zRotation)
  {
    return run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  @Override
  public void periodic()
  {
    leftMotorController.updateTelemetry();
    rightMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    leftMotorController.simIterate();
    rightMotorController.simIterate();
  }
}