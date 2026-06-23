// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

// TODO: Example with absolute encoders

/**
 * Exponentially profiled arm subsystem. The arm represented by this class does NOT have an absolute encoder! This
 * subsystem has a "self-homing" command, more details in the function description.
 *
 * <p>An exponential profile is chosen here instead of a trapezoidal profile because an arm
 * benefits from a velocity curve that accelerates quickly at large errors and decelerates
 * smoothly near the target. Trapezoidal profiles use fixed acceleration ramps, which can
 * produce an abrupt velocity change when the arm is close to the setpoint. The exponential
 * profile's velocity is proportional to the remaining error, so it blends naturally into the
 * PID controller's final correction without a hard deceleration edge.</p>
 */
public class ExponentiallyProfiledArmSubsystem extends SubsystemBase
{
  private final String           motorTelemetryName = "ExponentiallyProfiledArmMotor";
  private final String           mechTelemetryName  = "ExponentiallyProfiledArm";
  private final SparkMax         armMotor           = new SparkMax(1, MotorType.kBrushless);
  ///  Configuration Options
  private final DCMotor          dcMotor            = DCMotor.getNEO(1);
  // 7:1 reduction gives enough torque to hold a 10 lb arm against gravity while still
  // reaching reasonable angular velocity. Tune this to match your physical gearbox.
  private final MechanismGearing gearing            = new MechanismGearing(7);
  // 10 lbs is the estimated arm assembly mass (aluminum tube + game piece intake).
  // Update this when the final robot weight is known; it feeds the exponential constraint calc.
  private final Mass             weight             = Pounds.of(10);
  // 2 ft is the distance from the pivot axis to the arm's center of mass.
  // This drives the feedforward gravity term via createArmConstraints().
  private final Distance         length             = Feet.of(2);
  /*
   * Using the protractor, where 0deg on the protractor is when the arm is parallel to the ground,
   * you can measure where the starting angle should be.
   */
  // 30 deg is roughly where the arm rests after stowing at the start of a match.
  // The encoder is seeded to this value on boot because there is no absolute encoder.
  private final Angle            startingAngle      = Degrees.of(30);
  /*
   * To find these limits measure the starting angle relative to when the arm is parallel to the ground using a protractor.
   */
  // Soft limits stop the closed-loop controller before the arm reaches a hard stop.
  // -20 deg is the lowest safe commanded position (just above the frame perimeter).
  // 100 deg is the highest safe commanded position (just below the over-the-back hard stop).
  private final Angle            softLowerLimit     = Degrees.of(-20);
  private final Angle            softUpperLimit     = Degrees.of(100);
  /*
   * These are the real "limits" of the robot shown in simulation.
   */
  // Hard limits are physical bumpers; the sim uses them to stop the arm model.
  // They are set 10 deg outside the soft limits so simulation shows the consequence of
  // the arm escaping its soft-limit envelope before hitting the real mechanical stop.
  private final Angle            hardLowerLimit     = Degrees.of(-30);
  private final Angle            hardUpperLimit     = Degrees.of(110);

  /*
   * This is the STARTING PID Controller for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   *
   * kP=1 is a conservative starting value. With an exponential profile providing most of the
   * motion, kP only needs to close the small residual error at the setpoint. Increase it if the
   * arm lags at the end of a move; decrease it if it oscillates.
   * kI and kD both start at 0; add kI only if there is a persistent steady-state error under
   * gravity load that the feedforward does not eliminate.
   *
   * createArmConstraints() derives the profile speed limit from the motor's free-speed and the
   * gear ratio rather than requiring a manually guessed velocity limit. The 12 V ceiling matches
   * the nominal battery voltage under load.
   */
  private final ExponentialProfilePIDController pidController  = new ExponentialProfilePIDController(1,
                                                                                                     0,
                                                                                                     0,
                                                                                                     ExponentialProfilePIDController.createArmConstraints(
                                                                                                         Volts.of(12),
                                                                                                         dcMotor,
                                                                                                         weight,
                                                                                                         length,
                                                                                                         gearing));
  /*
   * This is the STARTING Feedforward for the Arm. If you are using a TalonFX or TalonFXS this will run on the motor controller itself.
   *
   * All four gains (kS, kG, kV, kA) start at 0. Run a SysId routine to characterize the real
   * arm and replace these values. kG in particular matters for an arm because the gravity
   * component changes with cos(angle); ArmFeedforward handles that automatically once kG != 0.
   */
  private final ArmFeedforward                  armFeedforward = new ArmFeedforward(0, 0, 0, 0);
  /**
   * {@link SmartMotorControllerConfig} for the arm motor.
   */
  private final SmartMotorControllerConfig      motorConfig    = new SmartMotorControllerConfig(this)
      /*
       * Basic Configuration options for the motor
       */
      .withMotorInverted(false)
      .withIdleMode(MotorMode.BRAKE) // BRAKE keeps the arm from sagging when commands stop.
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withGearing(gearing)
      .withStatorCurrentLimit(Amps.of(40)) // Prevents our motor from continuously over-taxing itself when it is stuck.
      .withClosedLoopRampRate(Seconds.of(0.25)) // Prevents our motor from rapid demand changes that could cause dramatic voltage drops, and current draw.
      .withOpenLoopRampRate(Seconds.of(0.25)) // Same as above
      .withTelemetry(motorTelemetryName,
                     TelemetryVerbosity.HIGH) // Could have more fine-grained control over what gets reported with SmartMotorControllerTelemetryConfig
      /*
       * Closed loop configuration options for the motor.
       *
       * withExponentialProfile() takes an ExponentialProfile.Constraints object computed by
       * createArmConstraints(). This is different from withTrapezoidalProfile(), which takes a
       * fixed max-velocity and max-acceleration pair. The exponential version derives both from
       * motor physics, so the profile automatically respects what the motor can actually deliver.
       */
      .withClosedLoopController(pidController.getP(), pidController.getI(), pidController.getD())
      .withExponentialProfile(pidController.getConstraints().get())
      .withFeedforward(armFeedforward)
      .withSoftLimits(softLowerLimit, softUpperLimit)
      .withStartingPosition(startingAngle);

  /// Generic Smart Motor Controller with out options and vendor motor.
  private final SmartMotorController motor    = new SparkWrapper(armMotor, dcMotor, motorConfig);
  /// Arm-specific options
  private       ArmConfig            m_config = new ArmConfig()
      /*
       * Basic configuration options for the arm.
       */
      .withLength(length)
      // Mass is fed to ExponentialProfilePIDController.createArmConstraints() above, not stored in ArmConfig.
      //.withStartingPosition(startingAngle) // The starting position should ONLY be defined if you are NOT using an absolute encoder. (moved to motorConfig)
      //.withHorizontalZero(Degrees.of(0)) // The horizontal zero should ONLY be defined if you ARE using an absolute encoder.
      .withTelemetry(mechTelemetryName, TelemetryVerbosity.HIGH)
      /*
       * Simulation configuration options for the arm.
       */
      .withHardLimits(hardLowerLimit, hardUpperLimit);
  // Arm mechanism
  private final Arm                  arm      = new Arm(m_config, motor);

  public ExponentiallyProfiledArmSubsystem()
  {
  }

  public void periodic()
  {
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  /**
   * Reset the encoder to the lowest position when the current threshold is reached. Should be used when the Arm
   * position is unreliable, like startup. Threshold is only detected if exceeded for 0.4 seconds, and the motor moves
   * less than 2 degrees per second.
   *
   * <p>Homing strategy: drive the arm toward hardUpperLimit at a fixed low voltage (2 V is
   * enough to overcome gravity but not enough to damage the mechanism). When stator current
   * stays above the caller-supplied threshold for 0.4 s while the arm is nearly still, the arm
   * has hit the hard stop. At that point, seed the encoder with hardUpperLimit so all future
   * closed-loop commands have an accurate reference frame.</p>
   *
   * @param threshold Current drawn when the arm is stalled against the hard stop. Start with
   *                  Amps.of(20) and reduce if the arm fails to trigger, or increase if it
   *                  triggers prematurely on normal motion resistance.
   * @return A Command that homes the arm and re-enables closed-loop control when finished.
   */
  public Command homing(Current threshold)
  {
    Debouncer       currentDebouncer  = new Debouncer(0.4); // Current threshold is only detected if exceeded for 0.4 seconds.
     Voltage        StopVolts         = Volts.of(0); // Volts to stop the homing routine.
    Voltage         runVolts          = Volts.of(2); // Low enough to limit stall current, high enough to move against gravity.
    Angle           limitHit          = hardUpperLimit;  // Limit which gets hit. Could be the lower limit if the volts makes the arm go down.
    AngularVelocity velocityThreshold = DegreesPerSecond.of(2); // The maximum amount of movement for the arm to be considered "hitting the hard limit".
    return Commands.startRun(motor::stopClosedLoopController, // Stop the closed loop controller
                             () -> motor.setVoltage(runVolts)) // Set the voltage of the motor
                   .until(() -> currentDebouncer.calculate(motor.getStatorCurrent().gte(threshold) &&
                                                           motor.getMechanismVelocity().abs(DegreesPerSecond) <=
                                                           velocityThreshold.in(DegreesPerSecond)))
                   .finallyDo(() -> {
                     motor.setVoltage(StopVolts);
                     motor.setEncoderPosition(limitHit); // Seed encoder so closed-loop commands are accurate from here on.
                     motor.startClosedLoopController();
                   });
  }

  /**
   * Drive the arm at a fixed duty cycle. Useful for manual override or open-loop testing.
   * Positive duty cycle moves the arm toward the upper limit; negative toward the lower limit.
   *
   * @param dutycycle Fraction of bus voltage to apply, in the range [-1.0, 1.0].
   * @return A Command that applies the duty cycle while active.
   */
  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  /**
   * Move the arm to a specific angle using the exponential profile closed-loop controller.
   * The profile generates a smooth velocity trajectory that converges quickly far from the
   * target and slows gently as the arm approaches the setpoint.
   *
   * @param angle The desired arm angle. Must be within the soft limits or the command will
   *              clamp to the nearest limit.
   * @return A Command that runs until the arm reaches the target angle.
   */
  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }
}
