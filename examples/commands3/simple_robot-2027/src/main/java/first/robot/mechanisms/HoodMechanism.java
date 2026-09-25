// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.mechanisms;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import org.wpilib.hardware.bus.CANPort;
import com.ctre.phoenix6.hardware.TalonFXS;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import yams.commands3.config.SmartMotorControllerConfig;
import yams.commands3.mechanisms.Pivot;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

public class HoodMechanism implements Mechanism
{
  private final TalonFXS                   hoodMotor        = new TalonFXS(9, new CANBus(CANPort.CAN_S0));//, MotorType.kBrushless);
  private final SmartMotorControllerConfig motorConfig      = new SmartMotorControllerConfig(this)
      .withClosedLoopController(4, 0, 0)
          .withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
          .withSoftLimits(Degrees.of(-30), Degrees.of(100))
          .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
          .withIdleMode(MotorMode.BRAKE)
          .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
          .withStatorCurrentLimit(Amps.of(40))
          .withMotorInverted(false)
          .withClosedLoopRampRate(Seconds.of(0.25))
          .withOpenLoopRampRate(Seconds.of(0.25))
          .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
          .withControlMode(ControlMode.CLOSED_LOOP)
          .withStartingPosition(Degrees.of(0));
  private final SmartMotorController motor =
      new TalonFXSWrapper(hoodMotor, DCMotor.getNEO(1), motorConfig);
  private final MechanismPositionConfig robotToMechanism =
      new MechanismPositionConfig()
          .withMaxRobotHeight(Meters.of(1.5))
          .withMaxRobotLength(Meters.of(0.75))
          .withRelativePosition(new Translation3d(Meters.of(-0.25), Meters.of(0), Meters.of(0.5)));
  private final PivotConfig m_config =
      new PivotConfig()
          .withHardLimits(Degrees.of(-100), Degrees.of(200))
          .withTelemetry("HoodExample", TelemetryVerbosity.HIGH)
          .withMechanismPositionConfig(robotToMechanism);
  private final Pivot hood = new Pivot(m_config, motor);

  public HoodMechanism() {
    // TODO: Set the default command, if any, for this mechanism by calling
    // setDefaultCommand(command)
    //       in the constructor or in the Robot class.
  }

  public void periodic() {
    hood.updateTelemetry();
  }

  public void simulationPeriodic() {
    hood.simIterate();
  }

  public Command hoodCmd(double dutycycle) {
    return hood.set(dutycycle);
  }

  public Command setAngle(Angle angle) {
    return hood.setAngle(angle);
  }

  public void setAngleSetpoint(Angle angle) {
    hood.setMechanismPositionSetpoint(angle);
  }
}
