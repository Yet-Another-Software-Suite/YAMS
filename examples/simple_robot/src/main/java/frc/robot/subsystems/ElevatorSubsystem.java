// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import org.wpilib.math.controller.ElevatorFeedforward;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.math.ExponentialProfilePIDController;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class ElevatorSubsystem extends SubsystemBase {
  private final Distance chainPitch = Inches.of(0.25);
  private final int toothCount = 22;
  private final Distance circumference = chainPitch.times(toothCount);
  private final Distance radius = circumference.div(2 * Math.PI);
  private final Mass weight = Pounds.of(16);
  private final DCMotor motors = DCMotor.getNEO(1);
  private final MechanismGearing gearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  private final SparkMax elevatorMotor = new SparkMax(2, SparkLowLevel.MotorType.kBrushless);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new
  //  SmartMotorControllerTelemetryConfig()
  //          .withMechanismPosition()
  //          .withRotorPosition()
  //          .withMechanismLowerLimit()
  //          .withMechanismUpperLimit(); // Specific telemetry verbosity
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
          .withMechanismCircumference(circumference)
          //      .withFollowers(Pair.of(new SparkMax(3, SparkLowLevel.MotorType.kBrushless), true))
          .withClosedLoopController(30, 0, 0)
          .withExponentialProfile(ExponentialProfilePIDController.createElevatorConstraints(
              Volts.of(12), motors, weight, radius, gearing))
          //      .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.5),
          //      MetersPerSecondPerSecond.of(0.5)) // Trapezoidal Profile PID Controller
          .withSoftLimits(Meters.of(0), Meters.of(2))
          .withGearing(gearing)
          //      .withExternalEncoder(armMotor.getAbsoluteEncoder()) // External Encoder if you
          //      need one, really shouldn't be used for Elevators
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
          //      .withSpecificTelemetry("ElevatorMotor", motorTelemetryConfig) // Specific
          //      Telemetry
          .withStatorCurrentLimit(Amps.of(40))
          //      .withVoltageCompensation(Volts.of(12)) // Voltage compensation isn't available on
          //      all controllers
          .withMotorInverted(false)
          //      .withClosedLoopRampRate(Seconds.of(0.25)) // Closed Loop Ramp Rate not necessary
          //      .withOpenLoopRampRate(Seconds.of(0.25)) // Open Loop Ramp Rate not necessary
          .withFeedforward(new ElevatorFeedforward(0, 0.1, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withStartingPosition(Meters.of(0.5));
  private final SmartMotorController motor = new SparkWrapper(elevatorMotor, motors, motorConfig);
  private final MechanismPositionConfig m_robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private ElevatorConfig m_config = new ElevatorConfig()
                                        .withHardLimits(Meters.of(0), Meters.of(3))
                                        .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
                                        .withMechanismPositionConfig(m_robotToMechanism)
                                        .withCarriageWeight(weight);
  private final Elevator m_elevator = new Elevator(m_config, motor);

  public ElevatorSubsystem() {
    new Trigger(() -> m_elevator.getHeight().lte(Meters.of(0.1)))
        .and(()
                 -> motor.getMechanismPositionSetpoint()
                     .orElse(Rotations.of(1))
                     .isEquivalent(Rotations.of(0)))
        .whileTrue(m_elevator.set(0));
  }

  public void periodic() {
    m_elevator.updateTelemetry();
  }

  public void simulationPeriodic() {
    m_elevator.simIterate();
  }

  public Command elevCmd(double dutycycle) {
    return m_elevator.set(dutycycle);
  }

  public Command setHeight(Distance height) {
    return m_elevator.setHeight(height);
  }
}
