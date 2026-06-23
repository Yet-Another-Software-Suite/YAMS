// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
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
 * Simple flywheel shooter subsystem demonstrating continuous closed-loop velocity control
 * with SimpleMotorFeedforward.
 *
 * <p>Unlike a positional mechanism (arm, elevator), a flywheel's setpoint is angular velocity,
 * and the closed-loop controller runs indefinitely to hold that speed against disturbances such
 * as ball loading drag.  A SimpleMotorFeedforward does most of the work; the PID term trims
 * steady-state error that the feedforward cannot model (e.g., manufacturing variance, belt
 * stretch, or load changes).
 *
 * <p>Mechanism summary:
 * <ul>
 *   <li>Motor:   REV NEO on a SparkMAX, CAN ID 1</li>
 *   <li>Gearing: 12:1 reduction (3-stage, 4:1 each)</li>
 *   <li>Wheel:   4-inch diameter, ~1 lb effective mass</li>
 *   <li>Top speed at 12 V: roughly 450 RPM at the wheel after reduction</li>
 * </ul>
 *
 * <p>Characterization constants (obtained via SysId on this exact mechanism):
 * <ul>
 *   <li>kS = 0.27937 V  -- static friction; volts needed before the shaft starts moving</li>
 *   <li>kV = 0.089836 V/RPS -- back-EMF slope; higher kV means more voltage per unit speed</li>
 *   <li>kA = 0.014557 V/(RPS/s) -- inertia term; small because the wheel is light</li>
 * </ul>
 */
public class ShooterSubsystem extends SubsystemBase
{
  /*
   * Physical hardware
   */
  // SparkMAX on CAN ID 1; NEO is brushless so kBrushless is required.
  private final SparkMax                   armMotor    = new SparkMax(1, MotorType.kBrushless);

  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();

  /*
   * Motor controller configuration
   */
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      /*
       * kP = 0.00016541: this looks tiny, but velocity error is measured in RPM (hundreds of
       * units), so even a small gain produces meaningful correction.  Contrast with a position
       * loop where error is in rotations (0-1 range) and kP would be 10-100x larger.
       */
      .withClosedLoopController(0.00016541, 0, 0)
      /*
       * 3 stages of 4:1 = 12:1 total reduction.  The wheel spins 12x slower than the rotor,
       * which trades top speed for torque -- useful when the wheel needs to accelerate a ball
       * quickly from a standing start.
       */
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      /*
       * COAST, not BRAKE: after a shot the wheel should spin down on its own rather than
       * actively decelerating.  Hard braking on a fast flywheel would spike current and stress
       * the gearbox.
       */
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      // 40 A stator limit: protects the NEO if the wheel is stalled or jammed.
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      /*
       * 0.25 s ramp rate on both open- and closed-loop paths limits the rate of voltage change
       * during spin-up.  Without this, demanding full speed from rest can pull enough current
       * to brown out the battery, especially if other mechanisms are active simultaneously.
       */
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      /*
       * SimpleMotorFeedforward(kS, kV, kA) is added to the PID output each loop.
       * kS = 0.27937 V  : static friction offset -- the controller applies this before any
       *                    velocity term so the motor does not stall at low setpoints.
       * kV = 0.089836 V/(rot/s): characterizes back-EMF; multiply by desired RPS to get the
       *                    baseline voltage for that speed.
       * kA = 0.014557 V/(rot/s^2): adds extra voltage during acceleration to overcome inertia;
       *                    small here because the 1-lb wheel has low rotational inertia.
       *
       * Both withFeedforward and withSimFeedforward use the same gains so simulation closely
       * matches real behavior during development.
       */
      .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
      /*
       * Moment-of-inertia estimate for simulation: YAMS approximates a solid disk from the
       * wheel radius (4-inch diameter -> 2-inch radius) and a 1-lb effective mass.
       * I = 0.5 * m * r^2 for a disk.  This drives how fast the simulated wheel
       * accelerates in response to voltage, which makes sim spin-up timing realistic.
       */
      .withMomentOfInertia(Inches.of(4), Pounds.of(1))
      .withControlMode(ControlMode.CLOSED_LOOP);

  // Wrap the hardware and config into a vendor-agnostic SmartMotorController.
  private final SmartMotorController       motor       = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);

  /*
   * FlyWheel mechanism config
   *
   * withDiameter: physical wheel diameter, used for linear-speed calculations in telemetry.
   * withSpeedometerSimulation: realistic free-spin speed used by the simulator when no load
   *   is present; 750 RPM at the wheel matches the expected no-load output at typical voltages
   *   through the 12:1 reduction.
   */
  private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH)
      .withSpeedometerSimulation(RPM.of(750));

  // FlyWheel combines the mechanism config with the motor controller.
  private final FlyWheel       shooter       = new FlyWheel(shooterConfig, motor);

  public ShooterSubsystem() {}

  /** Returns the current wheel angular velocity as measured by the motor encoder. */
  public AngularVelocity getVelocity() {return shooter.getSpeed();}

  /**
   * Runs the flywheel at a fixed target velocity. The closed-loop controller
   * and feedforward maintain this speed continuously until the command ends.
   *
   * @param speed Desired angular velocity at the wheel (after gearing).
   * @return A command that holds the given speed while scheduled.
   */
  public Command setVelocity(AngularVelocity speed) {return shooter.run(speed);}

  /**
   * Drives the flywheel in open-loop at a fixed duty cycle. Useful for manual
   * tuning or fallback if characterization data is unavailable.
   *
   * @param dutyCycle Output fraction in [-1, 1].
   * @return A command that applies the given duty cycle while scheduled.
   */
  public Command setDutyCycle(double dutyCycle) {return shooter.set(dutyCycle);}

  /**
   * Supplier-based velocity command, suitable for joystick-driven or dashboard-driven
   * speed control where the setpoint changes each loop iteration.
   *
   * @param speed Supplier of the desired angular velocity.
   * @return A command that continuously polls the supplier and updates the setpoint.
   */
  public Command setVelocity(Supplier<AngularVelocity> speed) {return shooter.run(speed);}

  /**
   * Supplier-based duty-cycle command, mirroring {@link #setVelocity(Supplier)} for
   * open-loop use cases.
   *
   * @param dutyCycle Supplier of the output fraction in [-1, 1].
   * @return A command that continuously polls the supplier.
   */
  public Command setDutyCycle(Supplier<Double> dutyCycle) {return shooter.set(dutyCycle);}

  @Override
  public void periodic() {
      // Push motor and mechanism telemetry to SmartDashboard / NetworkTables each loop.
      shooter.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
      // Advance the physics model one robot loop cycle (20 ms).
      shooter.simIterate();
  }
}
