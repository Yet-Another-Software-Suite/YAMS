// Copyright (c) 2025-2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms.doubleflywheel;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.util.Pair;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import java.util.function.Supplier;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * With 2 {@link SmartMotorController}s we can control a DoubleFlyWheelMechanism which can
 * accurately control arc of an object given tuning data.
 */
public class DoubleFlyWheelMechanism implements Mechanism {
  private SmartMotorControllerConfig lowerFlyWheelConfig =
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withIdleMode(MotorMode.COAST)
          //      .withWheelDiameter(Inches.of(4)) // Only needed to find the MPH of the flywheel
          // for fun.
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withMomentOfInertia(Inches.of(4), Pounds.of(2))
          .withClosedLoopController(
              1, 0,
              0) // You generally do not want a profile because its not a position controlled loop.
          .withFeedforward(new SimpleMotorFeedforward(0, 0, 0)) // Helps track changing RPM goals
          .withMotorInverted(false)
          .withTelemetry("LowerFlyWheel", TelemetryVerbosity.HIGH);

  private SmartMotorController lowerFlyWheel = new TalonFXWrapper(new TalonFX(4, new CANBus(CANPort.CAN_S0)),
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
      .withTelemetry("UpperFlyWheel", TelemetryVerbosity.HIGH);
  private SmartMotorController       upperflyWheel       = new TalonFXWrapper(new TalonFX(6, new CANBus(CANPort.CAN_S0)),
                                                                              DCMotor.getKrakenX60(1),
                                                                              upperFlyWheelConfig);


  public DoubleFlyWheelMechanism()
  {
    upperflyWheel.setupTelemetry();
    lowerFlyWheel.setupTelemetry();
  }

  /**
   * Get the linear speed (commonly used for KM/h or MPH) of the flywheels.
   *
   * @return (Lower Flywheel Speed, Upper Flywheel Speed)
   */
  public Pair<LinearVelocity, LinearVelocity> getLinearSpeed() {
    return Pair.of(lowerFlyWheel.getMeasurementVelocity(), upperflyWheel.getMeasurementVelocity());
  }

  /**
   * Set the duty cycle of the upper and lower flywheels.
   *
   * @param lower Lower duty cycle.
   * @param upper Upper duty cycle.
   * @return {@link Command}
   */
  public Command setDutyCycle(double lower, double upper) {
    return run(coroutine -> {
          lowerFlyWheel.stopClosedLoopController();
          upperflyWheel.stopClosedLoopController();
          while (true) {
            lowerFlyWheel.setDutyCycle(lower);
            upperflyWheel.setDutyCycle(upper);
            coroutine.yield();
          }
        })
        .whenCanceled(
            () -> {
              lowerFlyWheel.startClosedLoopController();
              upperflyWheel.startClosedLoopController();
            })
        .named("Set Duty Cycle (Double FlyWheel)");
  }

  /**
   * Set the voltage of the upper and lower flywheels.
   *
   * @param lower Lower voltage.
   * @param upper Upper voltage.
   * @return {@link Command}
   */
  public Command setVoltage(Voltage lower, Voltage upper) {
    return run(coroutine -> {
          lowerFlyWheel.stopClosedLoopController();
          upperflyWheel.stopClosedLoopController();
          while (true) {
            lowerFlyWheel.setVoltage(lower);
            upperflyWheel.setVoltage(upper);
            coroutine.yield();
          }
        })
        .whenCanceled(
            () -> {
              lowerFlyWheel.startClosedLoopController();
              upperflyWheel.startClosedLoopController();
            })
        .named("Set Voltage (Double FlyWheel)");
  }

  /**
   * Create a {@link Command} to set the speeds for the upper and lower flywheels
   * based off the distance to the goal.
   *
   * @param distanceToGoal Distance from the center of the robot to the goal on the XY plane.
   * @return {@link Command}
   */
  public Command setSpeedForDistance(Supplier<Distance> distanceToGoal) {
    return run(coroutine -> {
          while (true) {
            var lowerSpeeds =
                RPM.of(
                    DoubleFlyWheelConstants.distanceToRPM
                        .getFirst()
                        .get(distanceToGoal.get().in(Meters)));
            var upperSpeeds =
                RPM.of(
                    DoubleFlyWheelConstants.distanceToRPM
                        .getSecond()
                        .get(distanceToGoal.get().in(Meters)));
            lowerFlyWheel.setVelocity(lowerSpeeds);
            upperflyWheel.setVelocity(upperSpeeds);
            coroutine.yield();
          }
        })
        .named("Set Speed For Distance (Double FlyWheel)");
  }

  /**
   * Set the velocity directly.
   *
   * @param lower Lower supplier
   * @param upper Upper velocity supplier
   * @return {@link Command}
   */
  public Command setVelocity(Supplier<AngularVelocity> lower, Supplier<AngularVelocity> upper) {
    return run(coroutine -> {
          while (true) {
            lowerFlyWheel.setVelocity(lower.get());
            upperflyWheel.setVelocity(upper.get());
            coroutine.yield();
          }
        })
        .named("Set Velocity Supplier (Double FlyWheel)");
  }

  /**
   * Set the velocity directly.
   *
   * @param lower Lower Velocity
   * @param upper Upper velocity
   * @return {@link Command}
   */
  public Command setVelocity(AngularVelocity lower, AngularVelocity upper) {
    return run(coroutine -> {
          while (true) {
            lowerFlyWheel.setVelocity(lower);
            upperflyWheel.setVelocity(upper);
            coroutine.yield();
          }
        })
        .named("Set Velocity (Double FlyWheel)");
  }

  public void periodic() {
    lowerFlyWheel.updateTelemetry();
    upperflyWheel.updateTelemetry();
  }

  public void simulationPeriodic() {
    lowerFlyWheel.simIterate();
    upperflyWheel.simIterate();
  }
}
