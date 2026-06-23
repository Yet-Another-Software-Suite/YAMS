// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Inches;

import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import yams.gearing.MechanismGearing;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

/**
 * Tank-drive base for the 2026 Kitbot. Four NEOs in a 2+2 leader/follower layout
 * (two per side). OPEN_LOOP only -- no encoder-based velocity hold -- which is
 * fine for driver control where the human provides the outer feedback loop.
 *
 * <p>Motor IDs follow the Kitbot wiring convention:
 *   Left  leader=21, follower=22
 *   Right leader=24, follower=23
 *
 * <p>arcadeDrive is set as the default command so the drive always has a
 * safe fallback if no other command claims it.
 */
public class DiffDriveSubsystem extends SubsystemBase
{
  // 3:4 reduction -- Kitbot gearbox; two NEOs per side so DCMotor.getNEO(2) models
  // both motors on one side as a single gearbox input for simulation accuracy.
  private MechanismGearing gearing = new MechanismGearing(3, 4);

  // 4-inch wheels are the standard Kitbot wheel size.
  private Distance         wheelDiameter = Inches.of(4);

  private SparkMax leftMotor  = new SparkMax(21, SparkMax.MotorType.kBrushless);
  private SparkMax rightMotor = new SparkMax(24, SparkMax.MotorType.kBrushless);

  // Followers mirror the leader output. false = same direction as leader.
  private SparkMax leftFollowerMotor  = new SparkMax(22, SparkMax.MotorType.kBrushless);
  private SparkMax rightFollowerMotor = new SparkMax(23, SparkMax.MotorType.kBrushless);

  // Left side is inverted because the motors are physically mirrored on the
  // chassis -- positive voltage on both sides would otherwise spin them in
  // opposite directions.
  private SmartMotorControllerConfig leftMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(gearing)
      .withIdleMode(MotorMode.COAST) // COAST lets the robot roll to a stop; easier to push around when disabled
      .withMotorInverted(true)
      .withWheelDiameter(wheelDiameter)
      .withTelemetry("LeftMotorMain", TelemetryVerbosity.LOW)
      .withFollowers(Pair.of(leftFollowerMotor, false)); // follower not inverted relative to leader

  private SmartMotorControllerConfig rightMotorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(gearing)
      .withIdleMode(MotorMode.COAST)
      .withMotorInverted(false)
      .withWheelDiameter(wheelDiameter)
      .withTelemetry("RightMotorMain", TelemetryVerbosity.LOW)
      .withFollowers(Pair.of(rightFollowerMotor, false));

  // SparkWrapper wraps each leader; followers are handled internally by the config.
  // DCMotor.getNEO(2) tells the simulator two NEOs are driving this side.
  private SmartMotorController leftMotorController  = new SparkWrapper(leftMotor, DCMotor.getNEO(2), leftMotorConfig);
  private SmartMotorController rightMotorController = new SparkWrapper(rightMotor, DCMotor.getNEO(2), rightMotorConfig);

  // DifferentialDrive applies its own squaring and deadband to the duty-cycle inputs,
  // so raw controller values are safe to pass through without pre-processing.
  private DifferentialDrive drive = new DifferentialDrive(leftMotorController::setDutyCycle,
                                                          rightMotorController::setDutyCycle);

  public DiffDriveSubsystem()
  {
    // stop() as default prevents the motors from holding their last output
    // if the driver command ends unexpectedly.
    setDefaultCommand(stop());
  }

  public Command stop()
  {
    return run(drive::stopMotor);
  }

  /**
   * Independent left/right speed control. Useful for autonomous routines that
   * need precise per-side voltage, or for testing individual sides.
   */
  public Command tankDrive(DoubleSupplier left, DoubleSupplier right)
  {
    return run(() -> drive.tankDrive(left.getAsDouble(), right.getAsDouble()));
  }

  /**
   * Single-stick arcade drive. xSpeed drives forward/backward; zRotation
   * turns in place. This is the default teleop binding.
   */
  public Command arcadeDrive(DoubleSupplier xSpeed, DoubleSupplier zRotation)
  {
    return run(() -> drive.arcadeDrive(xSpeed.getAsDouble(), zRotation.getAsDouble()));
  }

  @Override
  public void periodic()
  {
    leftMotorController.updateTelemetry();
    rightMotorController.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    leftMotorController.simIterate();
    rightMotorController.simIterate();
  }
}
