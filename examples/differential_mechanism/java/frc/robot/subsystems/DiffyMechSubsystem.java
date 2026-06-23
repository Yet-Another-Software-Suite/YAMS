// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.positional.DifferentialMechanism;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

/**
 * Subsystem for a two-DOF differential (diffy) mechanism driven by two NEO motors.
 *
 * <p>A differential mechanism couples two motors so their combined rotation produces two
 * independent degrees of freedom without two independent gearboxes:
 * <ul>
 *   <li>Both motors spin the same direction  -> tilt (pitch up/down).</li>
 *   <li>Motors spin opposite directions      -> twist (roll/rotation about the output shaft).</li>
 * </ul>
 *
 * <p>Each motor gets its own SmartMotorControllerConfig because the effective load seen by
 * each motor differs depending on which DOF is being driven.  Tilt fights gravity on both
 * motors simultaneously; twist loads one motor against the other.
 *
 * <p>Key numbers at a glance:
 * <pre>
 *   Gearing       3 * 4 * 5 = 60:1 total reduction
 *   Max vel       180 deg/s at the output shaft
 *   Max accel     90 deg/s^2  (half of max vel -- gentle ramp for a wrist-mass load)
 *   Current limit 40 A stator  (NEO rated ~60 A peak, 40 A gives headroom without nuisance trips)
 *   Ramp rate     0.25 s       (limits dV/dt to avoid brown-outs on a shared PDP rail)
 *   Starting pos  tilt=90 deg, twist=0 deg
 * </pre>
 */
public class DiffyMechSubsystem extends SubsystemBase
{
  // -----------------------------------------------------------------------
  // Left motor -- CAN ID 1
  // -----------------------------------------------------------------------

  private final SparkMax                   leftMotor  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig leftConfig = new SmartMotorControllerConfig(this)
          // kP=16 was tuned empirically; high gain is workable here because the 60:1 reduction
          // dramatically damps the plant and the trapezoidal profile limits velocity error.
          .withClosedLoopController(16, 0, 0)
          // 180 deg/s output vel keeps wrist motion visible and controllable during tele-op.
          // 90 deg/s^2 accel is half the velocity limit -- avoids sharp demand spikes at move start.
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          // 3*4*5 = 60:1 reduction; three-stage box chosen to fit inside a wrist housing.
          // High reduction buys holding torque without a brake -- important when tilt fights gravity.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("LeftMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          // 40 A stator protects the NEO on sustained stall (e.g., mechanism jammed at hard stop).
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          // 0.25 s ramp prevents sudden current spikes that can sag the battery and trip PDP breakers.
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          // Feedforward coefficients are zero because the sim plant handles the dynamics.
          // On real hardware, characterize with SysId and fill in ks, kg, kv, ka.
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       leftSMC    = new SparkWrapper(leftMotor,
          DCMotor.getNEO(1),
          leftConfig);

  // -----------------------------------------------------------------------
  // Right motor -- CAN ID 2
  // The right config mirrors the left.  Both motors carry the same gearing
  // because the differential splits effort equally between the two DOFs.
  // -----------------------------------------------------------------------

  private final SparkMax                   rightMotor  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  private final SmartMotorControllerConfig rightConfig = new SmartMotorControllerConfig(this)
          // Same kP as left -- symmetric gearbox means symmetric closed-loop dynamics.
          .withClosedLoopController(16, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("RightMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          // Not inverted here -- DifferentialMechanism handles the sign convention internally
          // when it decomposes tilt/twist commands into per-motor setpoints.
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
  private final SmartMotorController       rightSMC    = new SparkWrapper(rightMotor,
                                                                          DCMotor.getNEO(1),
                                                                          rightConfig);

  // -----------------------------------------------------------------------
  // Mechanism config and instance
  // -----------------------------------------------------------------------

  private final DifferentialMechanismConfig config = new DifferentialMechanismConfig(leftSMC, rightSMC)
          // tilt=90 deg means the wrist starts pointing straight up (vertical neutral).
          // twist=0 deg is the roll home position (flat face forward).
          .withStartingPosition(Degrees.of(90), Degrees.of(0))
          // MOI: 0.3 m arm length, 4 lb end-effector mass -- drives sim inertia calculation.
          .withMOI(Meters.of(0.3), Pounds.of(4))
          .withTelemetry("DiffyMech", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  private final DifferentialMechanism diffy     = new DifferentialMechanism(config);

  public DiffyMechSubsystem()
  {
  }

  /**
   * Command the mechanism to a specific tilt and twist angle.
   * Uses closed-loop position control on both motors simultaneously.
   *
   * @param tilt  Desired pitch angle (positive = up from neutral).
   * @param twist Desired roll angle about the output shaft.
   */
  public Command setAngle(Angle tilt, Angle twist) {
    return diffy.setPosition(tilt, twist);
  }

  /**
   * Drive both DOFs in open-loop duty cycle.
   * Positive tilt spins both motors forward; positive twist spins them in opposition.
   *
   * @param tilt  Duty cycle for the tilt DOF  [-1.0, 1.0].
   * @param twist Duty cycle for the twist DOF [-1.0, 1.0].
   */
  public Command set(double tilt, double twist) {
    return diffy.set(tilt, twist);
  }

  public void periodic()
  {
    diffy.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    diffy.simIterate();
  }
}
