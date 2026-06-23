// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Simple single-jointed arm driven by a NEO through a 12:1 gearbox, controlled with
 * a trapezoidal motion profile and a proportional-only closed-loop controller.
 *
 * <p>This is the minimal YAMS arm example: no absolute encoder, no gravity feedforward,
 * duty-cycle override available alongside position commands. The arm angle is tracked
 * with the NEO's built-in relative encoder; the encoder position is considered valid
 * from power-on (no homing routine required for this demo).
 *
 * <p>Soft limits constrain commanded motion to -30..100 deg. Hard limits (-100..200 deg)
 * are the absolute physical stops modeled in simulation.
 */
public class ArmSubsystem extends SubsystemBase
{
  // NEO on CAN ID 1.  Change this to match your robot's CAN bus assignment.
  private final SparkMax armMotor = new SparkMax(1, SparkLowLevel.MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // kP=4 produces 4 V per radian of error (~0.07 V/deg). I and D are 0 because
      // the trapezoidal profile keeps velocity error small enough that integral
      // windup would add more instability than benefit, and the stiff gearbox
      // damps out oscillation that derivative would otherwise suppress.
      .withClosedLoopController(4, 0, 0)
      // Software-enforced range inside the physical hard stops.
      // -30 deg keeps the arm clear of the chassis; 100 deg is the intake clearance limit.
      .withSoftLimits(Degrees.of(-30), Degrees.of(100))
      // 3 * 4 = 12:1 total reduction. Trades rotational speed for the holding torque
      // needed to keep the arm from back-driving under load.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // BRAKE mode holds arm position when no voltage command is active,
      // preventing slow drift from gravity.
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
      // 40 A stator limit prevents sustained overcurrent if the arm stalls against a hard stop.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      // 0.25 s ramp rate limits how quickly the closed-loop output can slew.
      // Prevents large inrush current spikes when a new setpoint is commanded.
      .withClosedLoopRampRate(Seconds.of(0.25))
      // All feedforward gains are 0: no gravity compensation, no static friction term.
      // The P gain alone is sufficient for this lightweight arm at low speeds.
      // Set kG to approximately (arm_mass_kg * arm_length_m * 9.81) / 12 (gear ratio)
      // to improve position holding accuracy near horizontal.
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Sim starts with the arm at 0 deg (horizontal). This matches the physical
      // starting position assumed by the relative encoder on power-up.
      .withSimStartingPosition(Degrees.of(0));

  private final SmartMotorController motor = new SparkWrapper(armMotor,
                                                              DCMotor.getNEO(1),
                                                              motorConfig);

  private ArmConfig m_config = new ArmConfig()
      // 0.135 m arm length is used by the simulator to compute moment of inertia.
      // Measure from the pivot center to the arm's center of mass.
      .withLength(Meters.of(0.135))
      // Hard limits define the absolute range shown in simulation (-100..200 deg).
      // Physical hard stops on the real robot should be placed slightly inside these values.
      .withHardLimits(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH);

  private final Arm arm = new Arm(m_config, motor);

  public ArmSubsystem()
  {
  }

  @Override
  public void periodic()
  {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  /**
   * Drive the arm open-loop at the given duty cycle.
   * Useful for manual adjustment and as a safe default command.
   *
   * @param dutycycle Output fraction in [-1, 1]; positive moves arm in the positive direction.
   * @return Command that runs while held, stops when released.
   */
  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  /**
   * Move the arm to a fixed angle using the closed-loop controller.
   * The trapezoidal profile ramps velocity so the arm does not slam into the setpoint.
   *
   * @param angle Target angle. Must be within the soft limits (-30..100 deg).
   * @return Command that ends when the profile completes.
   */
  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }
}
