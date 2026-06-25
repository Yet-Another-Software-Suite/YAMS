// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
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
 * Flywheel shooter with AdvantageKit input logging. The key pattern here is that
 * velocity, setpoint, volts, and current all live in ShooterInputs rather than
 * being read directly from the motor. During replay the logger re-injects the
 * original sensor values so every downstream decision recomputes identically
 * without touching real hardware.
 */
public class ShooterSubsystem extends SubsystemBase {
  /*
   * ShooterInputs is the boundary of the "replay bubble". Fields here cross the
   * boundary into deterministic replay territory; anything the robot computes FROM
   * these values (setpoint decisions, shot-ready logic) is recomputed fresh each
   * replay. @AutoLog generates ShooterInputsAutoLogged at compile time -- it adds
   * the serialization code needed for Logger.processInputs() to stamp every field
   * with the current log timestamp.
   *
   * Track velocity AND setpoint here so replay can reconstruct whether the shooter
   * was actually at speed at the moment a game piece was fed -- critical for
   * diagnosing missed shots from a log.
   */
  @AutoLog
  public static class ShooterInputs {
    // Actual wheel speed read from the motor encoder each loop.
    public AngularVelocity velocity = DegreesPerSecond.of(0);
    // Active closed-loop setpoint; logged so replay knows what was being demanded.
    public AngularVelocity setpoint = DegreesPerSecond.of(0);
    // Bus voltage applied to the motor phase -- useful for diagnosing brownouts.
    public Voltage volts = Volts.of(0);
    // Stator current; combined with volts this reconstructs power draw in replay.
    public Current current = Amps.of(0);
  }

  private final ShooterInputsAutoLogged shooterInputs = new ShooterInputsAutoLogged();

  private final SparkMax armMotor = new SparkMax(20, MotorType.kBrushless);

  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      // kP=1 is a starting point; a flywheel typically needs little P because
      // SimpleMotorFeedforward carries most of the steady-state load.
      .withClosedLoopController(1, 0, 0)
      // 10000 RPM ceiling is above the NEO free-speed (~5800 RPM) so the profile
      // never saturates; 60 RPM/s ramp keeps the belt from jerking at spin-up.
      .withTrapezoidalProfile(RPM.of(10000), RPM.per(Second).of(60))
      // 3:4 box gives a 12:1 total reduction from motor to flywheel shaft.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
      // COAST: letting the wheel spin down naturally avoids back-driving the gearbox.
      .withIdleMode(MotorMode.COAST)
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH)
      // 40 A stator limit prevents the NEO from melting the winding during a jam.
      .withStatorCurrentLimit(Amps.of(40))
      .withMotorInverted(false)
      // kS/kV/kA are all zero here -- tune them with SysId before competition.
      .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
      // MOI estimated from a 4-inch, 1-lb disk; replace with CAD value when available.
      .withMomentOfInertia(Inches.of(4), Pounds.of(1))
      .withControlMode(ControlMode.CLOSED_LOOP);

  private final SmartMotorController motor = new SparkWrapper(armMotor, DCMotor.getNEO(1), motorConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
      // Diameter of the flywheel used for surface-speed calculations.
      .withDiameter(Inches.of(4))
      .withTelemetry("ShooterMech", TelemetryVerbosity.HIGH);

  private final FlyWheel shooter = new FlyWheel(shooterConfig, motor);

  /**
   * Populate shooterInputs from the SMC. Called at the top of periodic() so
   * Logger.processInputs() can stamp everything before any command reads it.
   * Reading velocity from shooterInputs (not directly from motor) is what makes
   * replay work -- during replay the logger overwrites these fields with the
   * original recorded values.
   */
  private void updateInputs() {
    shooterInputs.velocity = shooter.getSpeed();
    shooterInputs.setpoint = motor.getMechanismSetpointVelocity().orElse(RPM.of(0));
    shooterInputs.volts = motor.getVoltage();
    shooterInputs.current = motor.getStatorCurrent();
  }

  public ShooterSubsystem() {
  }

  /**
   * Gets the current velocity of the shooter.
   *
   * @return FlyWheel velocity.
   */
  public AngularVelocity getVelocity() {
    // Read from shooterInputs, not the motor directly -- this is what gets
    // replayed. If you read from motor here, replay will see live hardware data
    // instead of the logged value and your replay will diverge.
    return shooterInputs.velocity;
  }

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {
    // recordOutput logs the setpoint as a computed output -- it is NOT replayed.
    // During replay this line re-runs and writes the recomputed setpoint to the
    // new log, which lets you compare commanded vs. actual across replay runs.
    Logger.recordOutput("Shooter/Setpoint", speed);
    return shooter.run(speed);
  }

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command set(double dutyCycle) {
    Logger.recordOutput("Shooter/DutyCycle", dutyCycle);
    return shooter.set(dutyCycle);
  }

  public Command setVelocity(Supplier<AngularVelocity> speed) {
    return Commands.run(() -> {
      AngularVelocity v = speed.get();
      Logger.recordOutput("Shooter/Setpoint", v);
      motor.setVelocity(v);
    }, this);
  }

  public Command setDutyCycle(Supplier<Double> dutyCycle) {
    return shooter.set(() -> {
      Logger.recordOutput("Shooter/DutyCycle",
          dutyCycle.get());
      return dutyCycle.get();
    });
  }

  @Override
  public void simulationPeriodic() {
    shooter.simIterate();
  }

  @Override
  public void periodic() {
    // Order matters: populate inputs first, then processInputs stamps them with
    // the current timestamp. Any code that runs after processInputs sees values
    // that are consistent across real, sim, and replay modes.
    updateInputs();
    Logger.processInputs("Shooter", shooterInputs);
    shooter.updateTelemetry();
  }
}
