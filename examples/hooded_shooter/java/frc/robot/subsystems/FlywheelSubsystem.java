// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
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
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Velocity-controlled flywheel for the hooded shooter.
 *
 * <p>The flywheel is a 4-inch diameter, 1-lb polycarbonate disk driven by a NEO through a 12:1
 * two-stage reduction (3x4). Velocity is regulated by a PID loop on the SparkMax with a
 * SimpleMotorFeedforward that handles most of the steady-state work -- the P gain only corrects
 * residual error. A trapezoidal velocity profile limits spin-up rate so the motor does not sag
 * the bus voltage when accelerating from rest to full speed.
 *
 * <p>The key teaching point of this subsystem: feedforward carries the load; PID trims the error.
 * See kS, kV, kA comments below.
 *
 * <p>A linear-velocity overload (setRPM with MetersPerSecond) allows the caller to specify the
 * desired surface (exit) speed of the ball without worrying about wheel diameter.
 */
public class FlywheelSubsystem extends SubsystemBase
{
  // 4-inch wheel diameter used for the RPM <-> surface-speed conversion below.
  private final Distance flywheelDiameter = Inches.of(4);
  private final SparkMax flywheelMotor    = new SparkMax(1, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // P = 0.00016541 -- very small because feedforward handles ~95% of the output;
      // this term only corrects the last few RPM of steady-state error.
      // I and D are zero: integral windup risks are high on a flywheel with large set-point
      // steps, and derivative amplifies sensor noise on a fast-spinning wheel.
      .withClosedLoopController(0.00016541, 0, 0)
      // Max velocity 5000 RPM; accel limit 2500 RPS/s (reaches full speed in ~2 s).
      // The ramp keeps inrush current from sagging the 12V rail during spin-up.
      .withTrapezoidalProfile(RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
      // 3-stage, 4-stage reduction = 12:1 overall.  Increases torque and makes PID gains
      // smaller in magnitude (motor-shaft speed is 12x mechanism speed).
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // COAST on disable so the wheel spins down naturally instead of hard-braking.
      // Hard braking at high RPM dumps energy into the motor as heat.
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      // 40 A stator limit -- sized to protect the motor windings during extended stalls
      // without nuisance trips during normal spin-up transients.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      // 0.25 s ramp on both open and closed loop prevents sudden voltage steps that
      // would trip the PDP breaker or cause brownouts on the RIO.
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      // SimpleMotorFeedforward(kS, kV, kA):
      //   kS = 0.27937 V -- volts needed to overcome static friction and get the wheel
      //        turning at all.  Measured with SysId quasistatic routine.
      //   kV = 0.089836 V/(rot/s) -- volts per unit of angular velocity at steady state.
      //        Equivalent to back-EMF constant; lower = more efficient motor/gearing.
      //   kA = 0.014557 V/(rot/s^2) -- volts per unit of angular acceleration.
      //        Non-zero because the rotational inertia of the disk requires extra voltage
      //        during speed changes.  A flywheel without this term undershoots during ramps.
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      // Sim feedforward matches the real one so simulation velocity traces mirror real robot.
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      // withMomentOfInertia(diameter, mass) uses the solid-disk approximation:
      //   J = 0.5 * m * r^2
      //   J = 0.5 * (1 lb) * (2 in)^2 = ~0.00023 kg*m^2
      // This feeds the FlywheelSim so simulated spin-up time matches reality.
      .withMomentOfInertia(Inches.of(4), Pounds.of(1))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(flywheelMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig flywheelConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("FlywheelMech", TelemetryVerbosity.HIGH)
      // Speedometer simulation capped at 7500 RPM -- slightly above max command speed
      // so the simulated gauge does not peg at the top during spin-up transients.
      .withSpeedometerSimulation(RPM.of(7500));

  private final FlyWheel flywheel = new FlyWheel(flywheelConfig, motor);

  public FlywheelSubsystem()
  {
  }

  public AngularVelocity getVelocity()
  {
    return flywheel.getSpeed();
  }

  public Command setVelocity(AngularVelocity speed)
  {
    return flywheel.run(speed);
  }

  public Command setDutyCycle(double dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed)
  {
    return flywheel.run(speed);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle)
  {
    return flywheel.set(dutyCycle);
  }

  @Override
  public void periodic()
  {
    flywheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    flywheel.simIterate();
  }

  /**
   * Spin the flywheel to achieve the given ball exit (surface) velocity.
   *
   * <p>Converts linear surface speed to angular velocity: omega = v / r = v / (pi * d).
   * Callers pass MetersPerSecond so the physics model (range table, ballistics) can work
   * entirely in SI without caring about wheel diameter.
   *
   * @param speed Desired ball exit speed in meters per second.
   * @return Command that holds the flywheel at the corresponding RPM.
   */
  public Command setRPM(LinearVelocity speed)
  {
    return flywheel.run(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }

  /**
   * Direct (non-command) version of {@link #setRPM(LinearVelocity)}.
   *
   * <p>Used by ShootOnTheMoveCommand to update the flywheel setpoint every loop cycle
   * without scheduling a new Command on each iteration.
   *
   * @param speed Desired ball exit speed in meters per second.
   */
  public void setRPMDirect(LinearVelocity speed)
  {
    motor.setVelocity(RotationsPerSecond.of(speed.in(MetersPerSecond) / flywheelDiameter.times(Math.PI).in(Meters)));
  }
}
