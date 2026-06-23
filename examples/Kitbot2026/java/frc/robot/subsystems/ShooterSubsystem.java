// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
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
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Flywheel shooter for the 2026 Kitbot. One NEO drives a 3:4 reduction into a
 * 4-inch diameter wheel. Velocity is commanded in RPM via closed-loop control.
 *
 * <p>The feedforward is currently zeroed (kS=kV=kA=0). That means kP=1 is doing
 * all the work: error in RPM times 1 equals the voltage demand. At low setpoints
 * this is fine, but at high speeds (200+ RPM) the proportional term alone cannot
 * reach steady-state without residual error because it has no model of the back-EMF
 * or static friction. Tune kV first (voltage per RPM at steady state) to remove the
 * steady-state offset, then trim kP for transient response.
 */
public class ShooterSubsystem extends SubsystemBase
{
  // CAN ID 10. Single NEO in brushless mode.
  private final SparkMax                   ShooterMotor    = new SparkMax(10, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // kP=1, kI=0, kD=0. See class javadoc for the tradeoff vs a tuned feedforward.
      .withClosedLoopController(1, 0, 0)
      // 3:4 reduction -- same gearbox ratio as the drivetrain. fromReductionStages
      // takes individual stage ratios and multiplies them together.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // COAST so the flywheel spins down naturally after a command ends; BRAKE
      // at flywheel speeds would spike current and stress the gearbox.
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      // 40 A stator limit protects the NEO during the spin-up transient without
      // noticeably reducing top-speed performance at this gear ratio.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      // 0.25 s ramp rate limits the inrush current spike on spin-up. A faster ramp
      // is possible but increases the chance of tripping the PDP breaker mid-match.
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // Feedforward is zeroed until characterized. See class javadoc for implications.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // withMomentOfInertia approximates the flywheel MOI for the simulator using an
      // arm-length + mass model. Inches.of(4) is the wheel radius used as the "arm
      // length," and Pounds.of(1) is the estimated wheel mass. This is not physically
      // exact but gives the sim a plausible spin-up time so velocity control behavior
      // looks realistic before the real robot is available.
      .withMomentOfInertia(Inches.of(4), Pounds.of(1))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor         = new SparkWrapper(ShooterMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig       shooterConfig = new FlyWheelConfig()
      // 4-inch diameter matches the physical wheel; used by FlyWheel to convert
      // between surface speed and angular velocity for telemetry.
      .withDiameter(Inches.of(4))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private final FlyWheel             shooter       = new FlyWheel(shooterConfig, motor);

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
  public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {return shooter.set(dutyCycle);}

  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

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
}
