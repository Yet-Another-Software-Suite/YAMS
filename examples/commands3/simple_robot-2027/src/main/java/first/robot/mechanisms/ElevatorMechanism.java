// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;


import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.util.CANPorts;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.command3.Trigger;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Elevator;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.math.ExponentialProfilePIDController;
import yams.core.mechanisms.config.ElevatorConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class ElevatorMechanism implements Mechanism
{
  /// Height the elevator rests at when nothing else is commanding it.
  public static final Distance STOW_HEIGHT = Meters.of(0);
  /// How close the elevator must be to a target height to count as there.
  public static final Distance TOLERANCE   = Meters.of(0.02);

  private final Distance chainPitch = Inches.of(0.25);
  private final int toothCount = 22;
  private final Distance circumference = chainPitch.times(toothCount);
  private final Distance radius = circumference.div(2 * Math.PI);
  private final Mass     weight = Pounds.of(16);
  private final DCMotor  motors = DCMotor.getNEO(1);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final SparkMax                    elevatorMotor      = new SparkMax(CANPorts.fromBusId(1), 2, SparkLowLevel.MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new SmartMotorControllerTelemetryConfig()
//          .withMechanismPosition()
//          .withRotorPosition()
//          .withMechanismLowerLimit()
//          .withMechanismUpperLimit(); // Specific telemetry verbosity
  private final SmartMotorControllerConfig motorConfig        = new SmartMotorControllerConfig(this)
      .withMechanismCircumference(circumference)
//      .withFollowers(Pair.of(new SparkMax(3, SparkLowLevel.MotorType.kBrushless), true))
      .withClosedLoopController(30, 0, 0)
              .withExponentialProfile(ExponentialProfilePIDController
          .createElevatorConstraints(Volts.of(12),
                                     motors,
                                     weight,
                                     radius,
                                     gearing))
//      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5), MetersPerSecondPerSecond.of(0.5)) // Trapezoidal Profile PID Controller
      .withSoftLimits(Meters.of(0), Meters.of(2))
      .withGearing(gearing)
//      .withExternalEncoder(armMotor.getAbsoluteEncoder()) // External Encoder if you need one, really shouldn't be used for Elevators
      .withIdleMode(MotorMode.BRAKE)
      .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
//      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig) // Specific Telemetry
      .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12)) // Voltage compensation isn't available on all controllers
      .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25)) // Closed Loop Ramp Rate not necessary
//      .withOpenLoopRampRate(Seconds.of(0.25)) // Open Loop Ramp Rate not necessary
      .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
      .withControlMode(ControlMode.CLOSED_LOOP);
  private final SmartMotorController       motor              = new SparkWrapper(elevatorMotor,
                                                                                 motors,
                                                                                 motorConfig);
  private final MechanismPositionConfig    m_robotToMechanism = new MechanismPositionConfig()
      .withMaxRobotHeight(Meters.of(1.5))
      .withMaxRobotLength(Meters.of(0.75))
      .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private       ElevatorConfig             m_config           = new ElevatorConfig()
      .withHardLimits(Meters.of(0), Meters.of(3))
      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
      .withMechanismPositionConfig(m_robotToMechanism);
  private final Elevator                   m_elevator         = new Elevator(m_config, motor);

  public ElevatorMechanism()
  {
    new Trigger(()->m_elevator.getHeight().lte(Meters.of(0.1)))
        .and(()->motor.getMechanismPositionSetpoint().orElse(Rotations.of(1)).isEquivalent(Rotations.of(0)))
        .whileTrue(m_elevator.set(0));
  }

  public void periodic()
  {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    m_elevator.simIterate();
  }

  public Command elevCmd(double dutycycle)
  {
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height)
  {
    return m_elevator.setHeight(height);
  }

  /**
   * Hold the elevator at a height at the lowest priority, so any other elevator command can take over. Meant for
   * default commands.
   *
   * @param height Height to hold.
   * @return {@link Command} that holds the elevator at the height until another elevator command runs.
   */
  public Command hold(Distance height)
  {
    return run(coroutine -> coroutine.await(m_elevator.setHeight(height)))
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Elevator Hold " + height.in(Meters) + " m");
  }

  /**
   * Hold the elevator at {@link #STOW_HEIGHT}. This is the elevator's default command.
   *
   * @return {@link Command} that stows the elevator.
   */
  public Command stow()
  {
    return hold(STOW_HEIGHT);
  }

  /**
   * Move the elevator to a height and finish once it is within {@link #TOLERANCE}. The elevator's default command
   * takes over after this ends.
   *
   * @param height Height to move to.
   * @return {@link Command} that ends when the elevator reaches the height.
   */
  public Command moveTo(Distance height)
  {
    return m_elevator.runTo(height, TOLERANCE);
  }

  /**
   * Trigger that is true while the elevator is within {@link #TOLERANCE} of a height.
   *
   * @param height Height to check.
   * @return {@link Trigger} for the elevator being at the height.
   */
  public Trigger near(Distance height)
  {
    return m_elevator.near(height, TOLERANCE);
  }

}
