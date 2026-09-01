// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.plant.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.wpilibj.DigitalInput;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.SensorConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.motorcontrollers.simulation.Sensor;

public class ArmSubsystem extends SubsystemBase {
  private final CANcoder cancoder = new CANcoder(2);
  private final TalonFX armMotor = new TalonFX(1);
  //  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig = new
  //  SmartMotorControllerTelemetryConfig()
  //          .withMechanismPosition()
  //          .withRotorPosition()
  //          .withMechanismLowerLimit()
  //          .withMechanismUpperLimit();
  private final SmartMotorControllerConfig motorConfig =
      new SmartMotorControllerConfig(this)
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
          .withMomentOfInertia(Meters.of(0.135), Pounds.of(1.5));

  private final SmartMotorController motor =
      new TalonFXWrapper(armMotor, DCMotor.getKrakenX60(1), motorConfig);
  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(0.25), Meters.of(0), Meters.of(0.5)));

  private ArmConfig m_config = new ArmConfig()
                                   .withLength(Meters.of(0.135))
                                   .withHardLimits(Degrees.of(-100), Degrees.of(200))
                                   .withTelemetry("ArmExample", TelemetryVerbosity.HIGH)
                                   .withMechanismPositionConfig(robotToMechanism);
  private final Arm arm = new Arm(m_config, motor);

  private DigitalInput dio = new DigitalInput(0);
  private final Sensor coralSensor =
      new SensorConfig("CoralDetectorBeamBreak")
          .withField("Beam", dio::get, false)
          .withSimulatedValue("Beam", Seconds.of(3), Seconds.of(4), true)
          .withSimulatedValue("Beam", arm.isNear(Degrees.of(40), Degrees.of(2)), true)
          .getSensor();

  public ArmSubsystem() {}

  public boolean getBeamBreak() {
    return coralSensor.getAsBoolean("Beam");
  }

  public void periodic() {
    getBeamBreak();
    arm.updateTelemetry();
  }

  public void simulationPeriodic() {
    arm.simIterate();
  }

  public Command armCmd(double dutycycle) {
    return arm.set(dutycycle);
  }

  public Command setAngle(Angle angle) {
    return arm.setAngle(angle);
  }
}
