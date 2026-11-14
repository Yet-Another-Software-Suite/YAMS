// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import first.robot.Constants;
import java.util.Optional;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ADRCConfig;
import yams.core.math.ADRCConfig.SystemOrder;
import yams.core.math.ADRCController;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * A shooter flywheel run by an active disturbance rejection controller (ADRC) instead of a PID controller: a NEO
 * turning a 4 in, 1 lb wheel directly. The ADRC was written by FRC Team 8331.
 *
 * <p>A flywheel's velocity is a first order plant: a voltage accelerates it. The ADRC's observer estimates the
 * back-EMF, friction and the speed a game piece takes out of the wheel as one disturbance and cancels it, so the
 * shooter needs no kS or kV and recovers quickly after each shot.
 *
 * <p>With HIGH telemetry, the ADRC's settings are tunable live in place of kP, kI and kD, under
 * {@code /Tuning/ADRCShooter/ShooterMotor/closedloop/adrc}.
 */
public class ADRCShooterMechanism implements Mechanism {
  private static final MechanismGearing GEARING = new MechanismGearing(1);
  private static final DCMotor MOTOR = DCMotor.getNEO(1);

  private final SmartMotorController motor = new SparkWrapper(
      new SparkMax(CANPorts.fromBusId(1), Constants.SHOOTER_MOTOR_ID, MotorType.kBrushless),
      MOTOR,
      new SmartMotorControllerConfig(this)
          .withControlMode(ControlMode.CLOSED_LOOP)
          // The ADRC takes the place of kP, kI, kD and the feedforward.
          .withADRCController(new ADRCConfig()
              // A velocity loop on a flywheel is first order.
              .withSystemOrder(SystemOrder.FLYWHEEL)
              // Short, so the observer keeps up with the back-EMF it estimates; 0.15 s overshoots by about 45 RPM.
              .withSettlingTime(Seconds.of(0.2))
)
          // The output limit and ramp of every closed loop, the ADRC's too.
          .withClosedLoopControllerMaximumVoltage(Volts.of(12))
          .withGearing(GEARING)
          // b0, how much one volt accelerates the wheel, is derived from the motor, the gearing and this inertia.
          .withMomentOfInertia(Inches.of(4), Pounds.of(1))
          .withZeroPower(MotorMode.COAST)
          .withStatorCurrentLimit(Amps.of(40))
          .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH));

  private final FlyWheel shooter = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(4))
      .withTelemetry("ADRCShooter", TelemetryVerbosity.HIGH),
      motor);

  /** Coasts whenever nothing else is using the shooter. */
  @Override
  public Command idle() {
    return run(coroutine -> {
      shooter.setDutyCycleSetpoint(0);
      coroutine.park();
    }).withPriority(Command.LOWEST_PRIORITY).named("ADRCShooter.Coast");
  }

  /**
   * Spin the shooter at a speed until interrupted.
   *
   * @param speed Speed to hold.
   * @return {@link Command}
   */
  public Command spinUp(AngularVelocity speed) {
    // Starts the closed loop controller, which runs the ADRC on the SystemCore, if coasting stopped it.
    return shooter.run(speed);
  }

  /**
   * The shooter's speed.
   *
   * @return Speed of the shooter wheel.
   */
  public AngularVelocity getSpeed() {
    return shooter.getSpeed();
  }

  /**
   * The shooter's ADRC, built from its {@link ADRCConfig} by the motor controller.
   *
   * @return {@link ADRCController}, present once the motor controller is configured.
   */
  public Optional<ADRCController> getADRC() {
    return motor.getADRCController();
  }

  /** Publishes YAMS telemetry. Called from {@link first.robot.Robot#robotPeriodic()}. */
  public void periodic() {
    shooter.updateTelemetry();
  }

  /** Steps the YAMS simulation. Run in its own Notifier, started in {@link first.robot.Robot#simulationInit()}. */
  public void simulationPeriodic() {
    shooter.simIterate();
  }
}
