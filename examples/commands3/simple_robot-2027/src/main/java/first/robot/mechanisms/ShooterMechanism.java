// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import java.util.function.Supplier;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ShooterMechanism implements Mechanism
{
  /// How close the shooter must be to a target velocity to count as at speed.
  public static final AngularVelocity TOLERANCE = RPM.of(50);

  private final TalonFX                    flywheelMotor1         = new TalonFX(1, new CANBus(CANPort.CAN_S0));
  private final TalonFX                    flywheelMotor2         = new TalonFX(2, new CANBus(CANPort.CAN_S0));
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
  private final SmartMotorController       motor                  = new TalonFXWrapper(flywheelMotor1,
                                                                                       DCMotor.getNEO(2),
                                                                                       motorConfig);
  private final FlyWheelConfig             shooterConfig          = new FlyWheelConfig()
      // Diameter of the flywheel.
      .withDiameter(Inches.of(4))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);
  private final FlyWheel                   shooter                = new FlyWheel(shooterConfig, motor);

  public ShooterMechanism() {}

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
   * @return {@link Command}
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link Command}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}


  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

  /**
   * Set the shooter's surface velocity from a supplier, read every loop.
   *
   * @param speed Supplier of the surface (exit) velocity.
   * @return {@link Command}
   */
  public Command setLinearVelocity(Supplier<LinearVelocity> speed) {return shooter.run(speed);}

  /**
   * Hold the shooter at a velocity at the lowest priority, so any other shooter command can take over. Meant for
   * default commands.
   *
   * @param speed Velocity to hold.
   * @return {@link Command}
   */
  public Command hold(AngularVelocity speed)
  {
    return run(coroutine -> coroutine.await(shooter.run(speed)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Shooter Hold " + speed.in(RPM) + " RPM");
  }

  /**
   * Hold the shooter at 0 RPM. This is the shooter's default command.
   *
   * @return {@link Command}
   */
  public Command stop() {return hold(RPM.of(0));}

  /**
   * Trigger that is true while the shooter is within {@link #TOLERANCE} of a velocity.
   *
   * @param speed Velocity to check.
   * @return {@link Trigger}
   */
  public Trigger atSpeed(AngularVelocity speed) {return shooter.near(speed, TOLERANCE);}

  public void simulationPeriodic()
  {
    shooter.simIterate();
  }

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

  public void setVelocitySetpoint(AngularVelocity speed)
  {
    shooter.setMechanismVelocitySetpoint(speed);
  }

  public void setDutyCycleSetpoint(double dutyCycle)
  {
    shooter.setDutyCycleSetpoint(dutyCycle);
  }
}
