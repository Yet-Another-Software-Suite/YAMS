// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.KilogramSquareMeters;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import java.util.function.Supplier;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** The flywheel from the Shooter tutorial, with a speed that can change every loop. */
public class ShooterMechanism implements Mechanism {

  // What the motor controller should do: its control mode, gains and gearing.
  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // Feedback constants (PID), on the real robot and in simulation.
      // A flywheel holds a speed, so it gets no motion profile.
      .withClosedLoopController(0.2, 0, 0)
      .withSimClosedLoopController(0.2, 0, 0)
      // Feedforward constants, on the real robot and in simulation.
      .withFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      .withSimFeedforward(new SimpleMotorFeedforward(0, 0.19, 0))
      // Gearing from the motor rotor to the flywheel: one 1.5:1 stage. With it the flywheel tops
      // out near 3800 RPM, above every speed in the shot tables.
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(1.5)))
      // How hard the wheel is to spin up, used by the simulation: two 4 inch
      // wheels, 0.68 kg (1.5 lb) in all, as solid discs: 0.5 x 0.68 kg x (0.0508 m)^2.
      .withMomentOfInertia(KilogramSquareMeters.of(0.0009))
      .withMotorInverted(false)
      // Coast, so that the wheel spins down by itself instead of being braked to a stop.
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("ShooterMotor", TelemetryVerbosity.HIGH);

  // The vendor motor controller object: a SPARK MAX with CAN ID 15.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 15, MotorType.kBrushless);

  // The SmartMotorController applies the config to the SPARK MAX, which drives one NEO.
  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig shooterConfig = new FlyWheelConfig()
      // The diameter of the flywheel: the size of its drawing, and what turns RPM into surface speed.
      .withDiameter(Inches.of(4))
      .withTelemetry("Shooter", TelemetryVerbosity.HIGH);

  // The flywheel mechanism.
  private final FlyWheel shooter = new FlyWheel(shooterConfig, motor);

  /**
   * Follow a speed that changes every loop. {@code FlyWheel.run(Supplier)} reads the supplier every loop, which is how
   * the shot's speed reaches the flywheel. This command does not end on its own.
   */
  public Command track(Supplier<AngularVelocity> speed) {
    return shooter.run(speed);
  }

  /** Drive the flywheel with a duty cycle from -1 to 1. The closed loop is not used. */
  public Command set(double dutycycle) {
    return shooter.set(dutycycle);
  }

  /** Let the flywheel coast at the lowest priority, so that any other shooter command can interrupt it. */
  public Command idle() {
    return run(coroutine -> coroutine.await(shooter.set(0)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Shooter Idle");
  }

  /**
   * True while the flywheel is within a tolerance of a speed that changes every loop. {@code FlyWheel.near(...)} takes a
   * fixed speed, so it cannot follow a shot whose speed changes as the robot moves.
   */
  public Trigger atSpeed(Supplier<AngularVelocity> speed, AngularVelocity tolerance) {
    return new Trigger(() -> shooter.isNear(speed.get(), tolerance));
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    // Publishes the flywheel's telemetry, and applies anything changed through live tuning.
    shooter.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    // Runs the flywheel's physics simulation.
    shooter.simIterate();
  }
}
