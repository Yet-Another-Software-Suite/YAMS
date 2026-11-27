// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.system.DCMotor;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The roller that pushes a game piece into the flywheel. A roller in YAMS is a {@code FlyWheel}, even when it only
 * needs a fixed voltage, as in "How do I set up an intake roller?".
 */
public class FeederMechanism implements Mechanism {

  private final SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
      // Open loop: no PID or feedforward, the motor just gets a duty cycle or voltage.
      .withControlMode(ControlMode.OPEN_LOOP)
      // 3:1 from the motor to the roller.
      .withGearing(new MechanismGearing(3))
      // Roller size and mass, used by the simulation.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withMotorInverted(false)
      .withZeroPower(MotorMode.COAST)
      .withStatorCurrentLimit(Amps.of(40))
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

  // A SPARK MAX with CAN ID 16.
  private final SparkMax spark = new SparkMax(CANPort.CAN_S0, 16, MotorType.kBrushless);

  private final SmartMotorController motor = new SparkWrapper(spark, DCMotor.getNEO(1), smcConfig);

  private final FlyWheelConfig feederConfig = new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Feeder", TelemetryVerbosity.HIGH);

  private final FlyWheel feeder = new FlyWheel(feederConfig, motor);

  /** Push a game piece into the flywheel. Runs until interrupted. */
  public Command feed() {
    return feeder.setVoltage(Volts.of(8));
  }

  /** Stop the roller at the lowest priority, so any other feeder command can take over. */
  public Command idle() {
    return run(coroutine -> coroutine.await(feeder.set(0)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Feeder Idle");
  }

  /** Call this from Robot.robotPeriodic(), before the scheduler runs. */
  public void periodic() {
    feeder.updateTelemetry();
  }

  /** Robot.simulationInit() runs this in its own Notifier, every robot loop. */
  public void simulationPeriodic() {
    feeder.simIterate();
  }
}
