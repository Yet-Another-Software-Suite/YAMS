// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;


import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import org.wpilib.hardware.discrete.DigitalInput;

import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Arm;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.SensorConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.motorcontrollers.simulation.Sensor;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ArmMechanism implements Mechanism
{
  /// Angle the arm rests at when nothing else is commanding it.
  public static final Angle STOW_ANGLE   = Degrees.of(0);
  /// Angle the arm picks up game pieces at.
  public static final Angle PICKUP_ANGLE = Degrees.of(40);
  /// How close the arm must be to a target angle to count as there.
  public static final Angle TOLERANCE    = Degrees.of(2);

  private final CANcoder                   cancoder    = new CANcoder(2, new CANBus(CANPort.CAN_S0));
  private final TalonFX                    armMotor    = new TalonFX(1, new CANBus(CANPort.CAN_S0));
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0)
    .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
      .withSoftLimits(Degrees.of(-30), Degrees.of(100))
      .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ArmMotor", motorTelemetryConfig)
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
      .withMotorInverted(false)
      .withClosedLoopRampRate(Seconds.of(0.25))
      .withOpenLoopRampRate(Seconds.of(0.25))
      .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withExternalEncoder(cancoder)
      .withExternalEncoderGearing(0.5)
      .withUseExternalFeedbackEncoder(true)
      .withStartingPosition(Degrees.of(0))
      .withMomentOfInertia(Meters.of(0.135), Pounds.of(1));

  private final SmartMotorController    motor            = new TalonFXWrapper(armMotor,
                                                                              DCMotor.getKrakenX60(1),
                                                                              motorConfig);
  private final MechanismPositionConfig robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));

  private       ArmConfig m_config = new ArmConfig()
      .withLength(Meters.of(0.135))
      .withHardLimits(Degrees.of(-100), Degrees.of(200))
      .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(robotToMechanism);
  private final Arm       arm      = new Arm(m_config, motor);

  private       DigitalInput dio         = new DigitalInput(0);
  private final Sensor       coralSensor = new SensorConfig("CoralDetectorBeamBreak")
      .withField("Beam", dio::get, false)
      .withSimulatedValue("Beam", Seconds.of(3), Seconds.of(4), true)
      .withSimulatedValue("Beam", () -> arm.isNear(PICKUP_ANGLE, TOLERANCE), true)
      .getSensor();

  public ArmMechanism()
  {
  }

  public boolean getBeamBreak()
  {
    return coralSensor.getAsBoolean("Beam");
  }

  public void periodic()
  {
    getBeamBreak();
    arm.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle)
  {
    return arm.set(dutycycle);
  }

  public Command setAngle(Angle angle)
  {
    return arm.setAngle(angle);
  }

  /**
   * Hold the arm at an angle at the lowest priority, so any other arm command can take over. Meant for default
   * commands.
   *
   * @param angle Angle to hold.
   * @return {@link Command} that holds the arm at the angle until another arm command runs.
   */
  public Command hold(Angle angle)
  {
    return run(coroutine -> coroutine.await(arm.setAngle(angle)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Arm Hold " + angle.in(Degrees) + " deg");
  }

  /**
   * Hold the arm at {@link #STOW_ANGLE}. This is the arm's default command.
   *
   * @return {@link Command} that stows the arm.
   */
  public Command stow()
  {
    return hold(STOW_ANGLE);
  }

  /**
   * Move the arm to an angle and finish once it is within {@link #TOLERANCE}. The arm's default command takes over
   * after this ends.
   *
   * @param angle Angle to move to.
   * @return {@link Command} that ends when the arm reaches the angle.
   */
  public Command moveTo(Angle angle)
  {
    return arm.runTo(angle, TOLERANCE);
  }

  /**
   * Trigger that is true while the arm is within {@link #TOLERANCE} of an angle.
   *
   * @param angle Angle to check.
   * @return {@link Trigger} for the arm being at the angle.
   */
  public Trigger near(Angle angle)
  {
    return arm.near(angle, TOLERANCE);
  }

  /**
   * Move the arm to {@link #PICKUP_ANGLE} and hold it there until the beam break sees a game piece.
   *
   * @return {@link Command} that ends once a game piece is detected.
   */
  public Command pickUp()
  {
    return run(coroutine -> {
      // The forked hold is a child of this command, so it ends as soon as the beam break trips.
      coroutine.fork(arm.setAngle(PICKUP_ANGLE));
      coroutine.waitUntil(this::getBeamBreak);
    }).named("Arm Pick Up");
  }
}
