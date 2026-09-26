// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot.subsystems;

import static first.robot.Constants.IntakeConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/** Intake rollers driven by two brushed motors, open loop, as a YAMS {@link FlyWheel}. */
public class Intake extends SubsystemBase {
  private final SparkMax intakeLeader = new SparkMax(CANPorts.fromBusId(1), kLeaderId, MotorType.kBrushed);
  private final SparkMax intakeFollower = new SparkMax(CANPorts.fromBusId(1), kFollowerId, MotorType.kBrushed);

  private final SmartMotorControllerConfig intakeConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      // Gearing is only used for telemetry and simulation on this open loop roller.
      .withGearing(new MechanismGearing(1.0))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withFollowers(Pair.of(intakeFollower, false))
      // Simulation only: rough estimate of the rollers' inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withTelemetry("IntakeMotor", TelemetryVerbosity.HIGH);

  // DCMotor.getCIM(2) is an estimate for the two brushed motors; it only affects simulation.
  private final SmartMotorController intakeMotorController = new SparkWrapper(intakeLeader, DCMotor.getCIM(2), intakeConfig);

  // Diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel intake = new FlyWheel(new FlyWheelConfig()
      .withDiameter(Inches.of(2))
      .withTelemetry("Intake", TelemetryVerbosity.HIGH),
      intakeMotorController);

  /** Creates a new Intake. */
  public Intake() {
  }

  public Command moveIntake(double velocity) {
    return intake.set(velocity);
  }

  @Override
  public void periodic() {
    intake.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    intake.simIterate();
  }
}
