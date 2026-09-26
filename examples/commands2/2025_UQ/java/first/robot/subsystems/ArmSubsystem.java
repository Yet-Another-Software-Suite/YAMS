// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Unqualified Quokkas quokkas2025 (MIT, see LICENSE-UQ).

package first.robot.subsystems;

import static first.robot.Constants.ArmConstants.*;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.util.CANPorts;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.rotation.DutyCycleEncoder;
import org.wpilib.math.controller.ArmFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Arm;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Arm driven by two brushed motors, closed loop as a YAMS {@link Arm}. The motor encoder is seeded
 * from the through bore encoder on startup.
 */
public class ArmSubsystem extends SubsystemBase {
  private final SparkMax armLeader = new SparkMax(CANPorts.fromBusId(1), kLeaderId, MotorType.kBrushed);
  private final SparkMax armFollower = new SparkMax(CANPorts.fromBusId(1), kFollowerId, MotorType.kBrushed);

  private final SmartMotorControllerConfig armConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withGearing(new MechanismGearing(kGearRatio))
      .withClosedLoopController(armkP, armkI, armkD)
      // The original had no feedforward.
      .withFeedforward(new ArmFeedforward(0, 0, 0))
      // Targets are clamped to the arm's limits, as in the original.
      .withSoftLimits(armRearLimit, armFrontLimit)
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(true)
      .withFollowers(Pair.of(armFollower, true))
      // Simulation only: start at the coral intake position and a rough estimate of the arm's inertia.
      .withSimStartingPosition(positionIntakeCoral)
      .withMomentOfInertia(Inches.of(18), Pounds.of(8))
      .withTelemetry("ArmMotor", TelemetryVerbosity.HIGH);

  // DCMotor.getCIM(2) is an estimate for the two brushed motors; it only affects simulation.
  private final SmartMotorController armMotorController = new SparkWrapper(armLeader, DCMotor.getCIM(2), armConfig);

  // Length is an estimate; it only affects the simulation display.
  private final Arm arm = new Arm(new ArmConfig()
      .withHardLimits(armRearLimit, armFrontLimit)
      .withLength(Inches.of(18))
      .withTelemetry("Arm", TelemetryVerbosity.HIGH),
      armMotorController);

  // Through bore encoder on the arm shaft, on a DIO port as in the original.
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(kEncoderChannel);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // Seed the motor encoder from the through bore once, after waiting 1 s for its first readings
    // to settle. The simulated through bore reads 0, so the sim uses the starting position instead.
    if (RobotBase.isReal()) {
      Timer.delay(1.0);
      armMotorController.setEncoderPosition(Rotations.of(encoder.get()));
    }
  }

  public Command moveArmToPosition(Angle position) {
    return arm.setAngle(position);
  }

  @Override
  public void periodic() {
    arm.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    arm.simIterate();
  }
}
