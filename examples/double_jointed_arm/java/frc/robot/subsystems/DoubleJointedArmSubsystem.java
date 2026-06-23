// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

/**
 * Double-jointed arm subsystem demonstrating two independently controlled rotary joints
 * (proximal "lower" and distal "upper") driven by separate NEO/SparkMax pairs.
 *
 * <p>Key design points worth understanding:
 * <ul>
 *   <li>Each joint has its own SmartMotorControllerConfig and ArmConfig. They are composed
 *       into a single DoubleJointedArm, which handles inverse-kinematics for Cartesian
 *       setpoints and dispatches individual angle targets otherwise.</li>
 *   <li>Both joints share the same gear reduction and profile constraints in this template.
 *       In a real mechanism the proximal joint carries the weight of the entire distal
 *       segment plus any game piece, so it typically needs a higher reduction or lower
 *       profile velocity than the distal joint.</li>
 *   <li>Soft limits are left commented out intentionally -- uncomment and tune them once
 *       you know the mechanical range of your specific arm to prevent self-collision.</li>
 *   <li>The feedforward gains are all zero here because they must be identified per robot.
 *       Run SysId or use ReCalc to produce real ks/kg/kv/ka values before competition.</li>
 * </ul>
 */

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.DoubleJointedArm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;

public class DoubleJointedArmSubsystem extends SubsystemBase
{
  // -------------------------------------------------------------------------
  // LOWER (PROXIMAL) JOINT
  // The proximal joint rotates the entire distal segment plus anything
  // attached to it. Motor is on CAN ID 1.
  // -------------------------------------------------------------------------

  private final SparkMax                   lowerMotor  = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
          // kP=16 produces roughly 16 volts of correction per radian of error.
          // This is a starting point; tune down if you see oscillation at the setpoint.
          .withClosedLoopController(16, 0, 0)
          // 180 deg/s velocity cap limits how fast the proximal joint can swing.
          // 90 deg/s^2 acceleration keeps the motion smooth and reduces current spikes.
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100)) // Uncomment once arm limits are measured
          // 3:4:5 staged reduction -> 60:1 total. High reduction maximises holding torque,
          // important for the proximal joint which must support the distal segment's weight.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder()) // Uncomment for absolute position on power-cycle
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE) // BRAKE prevents the joint from sagging when disabled
          .withTelemetry("LowerMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          // 40 A stator limit protects the NEO from stalling into a hard stop.
          // A NEO can sustain ~40 A continuously without thermal shutdown on a typical arm.
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          // 0.25 s ramp prevents sudden voltage steps that would spike bus current and
          // stress the gearbox on direction reversals.
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          // All feedforward terms are zero -- replace with SysId results before use.
          // ks: static friction voltage, kg: gravity compensation at horizontal,
          // kv: velocity feedforward, ka: acceleration feedforward.
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
          // 45 deg starting position relative to horizontal. Adjust to match the arm's
          // actual resting angle so the encoder agrees with reality at robot enable.
          .withStartingPosition(Degrees.of(45));

  private final SmartMotorController       lowerSMC    = new SparkWrapper(lowerMotor,
          DCMotor.getNEO(1),
          lowerConfig);

  private final ArmConfig        lowerArmConfig = new ArmConfig()
          // 2 ft segment length feeds into the simulation's moment-of-inertia calculation.
          .withLength(Feet.of(2))
          // Hard limits are wide (+/- 720 deg) to let the simulation run freely while
          // you are prototyping. Tighten these to real mechanical hard stops before
          // deploying to a physical robot so the sim reflects actual risk.
          .withHardLimits(Degrees.of(-720), Degrees.of(720))
          .withTelemetry("LowerArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);

  // -------------------------------------------------------------------------
  // UPPER (DISTAL) JOINT
  // The distal joint carries only the end-effector. It is lighter and faster
  // than the proximal joint. Motor is on CAN ID 2.
  // -------------------------------------------------------------------------

  private final SparkMax                   upperMotor  = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig upperConfig = new SmartMotorControllerConfig(this)
          // Same kP as lower joint for this template. In practice the distal joint
          // may tolerate a higher kP because it has less load and less inertia.
          .withClosedLoopController(16, 0, 0)
          // Profile matches the lower joint here. The distal joint could use a higher
          // velocity limit (e.g., 270 deg/s) since it moves a lighter load.
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          //.withSoftLimits(Degrees.of(-30), Degrees.of(100)) // Uncomment once arm limits are measured
          // Same 60:1 reduction as lower joint. Reduce this for faster tip speed
          // if the distal segment does not need the same holding torque.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
          .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
          .withTelemetry("UpperMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40)) // Same 40 A limit; distal joint can often tolerate 30 A in practice
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0)) // Replace with SysId results
          .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
          .withStartingPosition(Degrees.of(45)); // Match the upper segment's physical resting angle

  private final SmartMotorController       upperSMC    = new SparkWrapper(upperMotor,
                                                                          DCMotor.getNEO(1),
                                                                          upperConfig);

  private final ArmConfig        upperArmConfig = new ArmConfig()
      // 2.5 ft upper segment is longer than the lower segment, so the tip travels farther
      // per degree of rotation. This affects the IK solution for Cartesian setpoints.
      .withLength(Feet.of(2.5))
      .withHardLimits(Degrees.of(-720), Degrees.of(720)) // Widen or tighten to match physical stops
      .withTelemetry("UpperArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
      .withSimColor(new Color8Bit(Color.kDarkRed)); // Dark red distinguishes the upper segment in simulation

  // DoubleJointedArm wires both joints together. It exposes angle, duty-cycle,
  // and Cartesian (x, y) setpoint commands.
  private final DoubleJointedArm jointedArm     = new DoubleJointedArm(lowerArmConfig, lowerSMC, upperArmConfig, upperSMC);

  public DoubleJointedArmSubsystem()
  {
  }

  /**
   * Move the arm tip to a Cartesian (x, y) position in the plane of rotation.
   *
   * @param x             Horizontal distance from the shoulder pivot, positive forward.
   * @param y             Vertical height from the shoulder pivot, positive up.
   * @param elbowRequest  true selects the "elbow up" IK solution; false selects "elbow down".
   *                      Both solutions reach the same tip point -- choose based on what
   *                      avoids colliding with the robot frame or the field element.
   */
  public Command setPosition(Distance x, Distance y, boolean elbowRequest)
  {
    return jointedArm.setPosition(new Translation2d(x.in(Meters), y.in(Meters)), elbowRequest);
  }

  /**
   * Command both joints to explicit angle targets simultaneously.
   * Pass null for a joint to leave it at its current setpoint.
   *
   * @param lowerAngle Target angle for the proximal joint, measured from horizontal.
   * @param upperAngle Target angle for the distal joint, measured from horizontal.
   */
  public Command setAngle(Angle lowerAngle, Angle upperAngle) {
    return jointedArm.setAngle(lowerAngle, upperAngle);
  }

  /**
   * Drive both joints in open-loop duty cycle.
   * Pass null for a joint to leave it under its current control mode.
   *
   * @param lowerDutycycle Duty cycle for the proximal joint (-1.0 to 1.0).
   * @param upperDutycycle Duty cycle for the distal joint (-1.0 to 1.0).
   */
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

}
