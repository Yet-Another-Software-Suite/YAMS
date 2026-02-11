package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.thethriftybot.devices.ThriftyNova;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;

public class ShooterSubsystem extends SubsystemBase
{

  private final ThriftyNova                flywheelMotor1         = new ThriftyNova(1);
  private final TalonFX                    flywheelMotor2         = new TalonFX(2);
  private final boolean                    flywheelMotor2Inverted = true;
  private final SmartMotorControllerConfig motorConfig            = new SmartMotorControllerConfig(this)
      .withClosedLoopController(1, 0, 0)
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
//      .withVendorConfig(new TalonFXConfiguration().withVoltage(new VoltageConfigs().withPeakReverseVoltage(0)))
//      .withFollowers(Pair.of(flywheelMotor2, flywheelMotor2Inverted))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor                  = new NovaWrapper(flywheelMotor1,
                                                                                    DCMotor.getNEO(2),
                                                                                    motorConfig);
  private final FlyWheelConfig             shooterConfig          = new FlyWheelConfig(motor)
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      // Mass of the flywheel.
      .withMass(Pounds.of(4))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  private final FlyWheel                   shooter                = new FlyWheel(shooterConfig);

  public ShooterSubsystem() {}

  /**
   * Gets the current velocity of the shooter.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.setSpeed(speed);}

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}


  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

  @Override
  public void simulationPeriodic()
  {
    shooter.simIterate();
  }

  @Override
  public void periodic()
  {
    shooter.updateTelemetry();
  }

  public void setRPM(LinearVelocity newHorizontalSpeed)
  {
    shooter.setMeasurementVelocitySetpoint(newHorizontalSpeed);
  }

  public boolean readyToShoot(AngularVelocity tolerance)
  {
    if (motor.getMechanismSetpointVelocity().isEmpty())
    {return false;}
    return motor.getMechanismVelocity().isNear(motor.getMechanismSetpointVelocity().orElseThrow(), tolerance);
  }
}
