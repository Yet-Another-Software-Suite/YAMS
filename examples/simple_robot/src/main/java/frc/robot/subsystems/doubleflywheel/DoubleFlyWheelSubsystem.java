// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems.doubleflywheel;


import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.Pair;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import java.util.function.Supplier;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * With 2 {@link SmartMotorController}s we can control a DoubleFlyWheelSubsystem which can accurately control arc of an
 * object given tuning data.
 *
 */
public class DoubleFlyWheelSubsystem extends SubsystemBase
{
  private SmartMotorControllerConfig lowerFlyWheelConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
//      .withWheelDiameter(Inches.of(4)) // Only needed to find the MPH of the flywheel for fun.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMomentOfInertia(Inches.of(4), Pounds.of(2))
      .withClosedLoopController(1,
                                0,
                                0) // You generally do not want a profile because its not a position controlled loop.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // Helps track changing RPM goals
      .withMotorInverted(false)
      .withTelemetry("LowerFlyWheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);


  private SmartMotorController lowerFlyWheel = new TalonFXWrapper(new TalonFX(4),
                                                                  DCMotor.getKrakenX60(1),
                                                                  lowerFlyWheelConfig);

  private SmartMotorControllerConfig upperFlyWheelConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withIdleMode(MotorMode.COAST)
//      .withWheelDiameter(Inches.of(4)) // Only needed to find the MPH of the flywheel for fun.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      .withMomentOfInertia(Inches.of(4), Pounds.of(2))
      .withClosedLoopController(1,
                                0,
                                0) // You generally do not want a profile because its not a position controlled loop.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // Helps track changing RPM goals
      .withMotorInverted(false)
      .withTelemetry("UpperFlyWheel", SmartMotorControllerConfig.TelemetryVerbosity.HIGH);
  private SmartMotorController       upperflyWheel       = new TalonFXWrapper(new TalonFX(6),
                                                                              DCMotor.getKrakenX60(1),
                                                                              upperFlyWheelConfig);


  public DoubleFlyWheelSubsystem()
  {
    upperflyWheel.setupTelemetry();
    lowerFlyWheel.setupTelemetry();
  }

  /**
   * Get the linear speed (commonly used for KM/h or MPH) of the flywheels.
   *
   * @return (Lower Flywheel Speed, Upper Flywheel Speed)
   */
  public Pair<LinearVelocity, LinearVelocity> getLinearSpeed()
  {
    return Pair.of(lowerFlyWheel.getMeasurementVelocity(), upperflyWheel.getMeasurementVelocity());
  }

  /**
   * Set the duty cycle of the upper and lower flywheels.
   *
   * @param lower Lower duty cycle.
   * @param upper Upper duty cycle.
   * @return {@link org.wpilib.command2.RunCommand}
   */
  public Command setDutyCycle(double lower, double upper)
  {
    return startRun(() -> {
      lowerFlyWheel.stopClosedLoopController();
      upperflyWheel.stopClosedLoopController();
    }, () -> {
      lowerFlyWheel.setDutyCycle(lower);
      upperflyWheel.setDutyCycle(upper);
    }).finallyDo(() -> {
      lowerFlyWheel.startClosedLoopController();
      upperflyWheel.startClosedLoopController();
    }).withName("Set Duty Cycle (Double FlyWheel)");
  }

  /**
   * Set the voltage of the upper and lower flywheels.
   *
   * @param lower Lower voltage.
   * @param upper Upper voltage.
   * @return {@link org.wpilib.command2.RunCommand}
   */
  public Command setVoltage(Voltage lower, Voltage upper)
  {
    return startRun(() -> {
      lowerFlyWheel.stopClosedLoopController();
      upperflyWheel.stopClosedLoopController();
    }, () -> {
      lowerFlyWheel.setVoltage(lower);
      upperflyWheel.setVoltage(upper);
    }).finallyDo(() -> {
      lowerFlyWheel.startClosedLoopController();
      upperflyWheel.startClosedLoopController();
    }).withName("Set Voltage (Double FlyWheel)");
  }

  /**
   * Create a {@link org.wpilib.command2.RunCommand} to set the speeds for the upper and lower flywheels
   * based off the distance to the goal.
   *
   * @param distanceToGoal Distance from the center of the robot to the goal on the XY plane.
   * @return {@link org.wpilib.command2.RunCommand}
   */
  public Command setSpeedForDistance(Supplier<Distance> distanceToGoal)
  {
    return run(() -> {
      var lowerSpeeds = RPM.of(DoubleFlyWheelConstants.distanceToRPM.getFirst()
                                                                    .get(distanceToGoal.get().in(Meters)));
      var upperSpeeds = RPM.of(DoubleFlyWheelConstants.distanceToRPM.getSecond()
                                                                    .get(distanceToGoal.get().in(Meters)));
      lowerFlyWheel.setVelocity(lowerSpeeds);
      upperflyWheel.setVelocity(upperSpeeds);
    }).withName("Set Speed For Distance (Double FlyWheel)");
  }

  /**
   * Set the velocity directly.
   *
   * @param lower Lower supplier
   * @param upper Upper velocity supplier
   * @return {@link org.wpilib.command2.RunCommand}
   */
  public Command setVelocity(Supplier<AngularVelocity> lower, Supplier<AngularVelocity> upper)
  {
    return run(() -> {
      lowerFlyWheel.setVelocity(lower.get());
      upperflyWheel.setVelocity(upper.get());
    });
  }

  /**
   * Set the velocity directly.
   *
   * @param lower Lower Velocity
   * @param upper Upper velocity
   * @return {@link org.wpilib.command2.RunCommand}
   */
  public Command setVelocity(AngularVelocity lower, AngularVelocity upper)
  {
    return run(() -> {
      lowerFlyWheel.setVelocity(lower);
      upperflyWheel.setVelocity(upper);
    });
  }

  @Override
  public void periodic()
  {
    lowerFlyWheel.updateTelemetry();
    upperflyWheel.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    lowerFlyWheel.simIterate();
    upperflyWheel.simIterate();
  }
}
