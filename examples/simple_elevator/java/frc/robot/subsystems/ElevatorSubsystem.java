// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

/**
 * Single-stage elevator subsystem using a trapezoidal motion profile.
 *
 * <p>The carriage travels vertically on a continuous loop driven by a single NEO through a 12:1
 * two-stage gearbox and a 0.25-inch pitch chain on a 22-tooth sprocket. All setpoints are in
 * meters (linear distance), not rotations. The trapezoidal profile limits carriage velocity and
 * acceleration so the robot does not tip or jerk the load during fast moves.
 *
 * <p>Soft limits keep the closed-loop controller from demanding travel outside the safe mechanical
 * range. Hard limits in ElevatorConfig define the simulation bounding box and represent where
 * physical hard stops would be on a real robot.
 */

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase
{
  // CAN ID 2 is the elevator motor on this robot's CAN bus layout.
  private final SparkMax                   elevatorMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);

  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();

  private final SmartMotorControllerConfig motorConfig   = new SmartMotorControllerConfig(this)
      // Drum circumference = pi * d = pitch * teeth = 0.25 in * 22 teeth, converted to meters.
      // This is how YAMS converts encoder rotations into carriage displacement. Every full motor
      // revolution moves the carriage exactly one drum circumference up or down the travel path.
      .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
      // kP = 4 means 4 V/m of position error. With zero feedforward, kP alone must overcome
      // gravity at steady state, so it is intentionally aggressive here. Expect ~0.05 m of
      // steady-state sag when holding a height without feedforward; tune kG in ElevatorFeedforward
      // to eliminate that sag without increasing kP further.
      .withClosedLoopController(4, 0, 0)
      // 0.5 m/s and 0.5 m/s^2 are conservative for a light prototype elevator. A competition
      // robot carrying a game piece typically runs 2-4 m/s with 4-8 m/s^2 once gearing and
      // weight are finalized. Lower these first, then increase once the mechanism is proven.
      .withTrapezoidalProfile(MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5))
      // Soft limits match the usable range [0, 2 m] and are tighter than the hard limits [0, 3 m]
      // so the closed-loop controller slows down before hitting a physical stop.
      .withSoftLimits(Meters.of(0), Meters.of(2))
      // 3 * 4 = 12:1 total reduction. Two reduction stages (3:1 then 4:1) are typical for a
      // single-stage elevator to keep top speed reasonable at NEO free speed (~5600 RPM).
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      // BRAKE mode prevents the carriage from back-driving under gravity when disabled or idle.
      // Switching to COAST would let the carriage drift, which is unsafe for an elevator holding
      // a load above the ground.
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig)
      // 40 A stator limit protects the NEO from sustained overcurrent during stall (e.g., jammed
      // carriage or end-of-travel before the soft limit takes effect). NEO continuous rating is
      // ~40 A; stay at or below this to avoid winding damage.
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
      // All feedforward gains are zero here so the example is easy to run out of the box.
      // On a real elevator, kG (gravity constant) is critical: set it to the voltage required
      // to hold the carriage stationary at mid-travel. kV (velocity constant) reduces lag
      // during profiled moves. kS (static friction) helps the carriage break away cleanly.
      .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
      // Starting height: 0.5 m above the floor. This seeds the encoder so the closed-loop
      // controller knows where the carriage is before a homing routine runs.
      .withStartingPosition(Meters.of(0.5))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController       motor         = new SparkWrapper(elevatorMotor,
                                                                            DCMotor.getNEO(1),
                                                                            motorConfig);

  // robotToMechanism places the elevator in the simulation 3D model.
  // x = -0.25 m is 25 cm behind the robot center, z = 0.5 m is the base of the carriage travel.
  private final MechanismPositionConfig m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));

  private       ElevatorConfig          m_config           = new ElevatorConfig()
      // Hard limits define the simulation bounding box -- the range ElevatorSim uses to clamp
      // carriage position. 3 m is larger than the soft limit (2 m) so the sim can show the
      // carriage reaching the physical stop in the event a soft limit is disabled or bypassed.
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(m_robotToMechanism)
      // 16 lb (7.3 kg) is a rough estimate for a single-stage elevator with carriage, chain, and
      // a light payload. ElevatorSim uses this to calculate the gravitational force the motor
      // must oppose. A heavier estimate makes simulation more conservative -- the motor works
      // harder to hold position, which surfaces feedforward deficiencies earlier.
      .withCarriageWeight(Pounds.of(16));

  private final Elevator                m_elevator         = new Elevator(m_config, motor);

  public ElevatorSubsystem()
  {
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  /**
   * Open-loop duty-cycle command for manual control or testing.
   * Positive duty cycle raises the carriage; negative lowers it.
   *
   * @param dutycycle Fraction of bus voltage, [-1.0, 1.0].
   * @return Command that runs until interrupted.
   */
  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  /**
   * Closed-loop height command. Profiles to the target using the trapezoidal constraints defined
   * in motorConfig, then holds the carriage at that height via the PID controller.
   *
   * @param height Target carriage height in meters, clamped by soft limits [0, 2 m].
   * @return Command that runs until the carriage reaches and holds the setpoint.
   */
  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

}
