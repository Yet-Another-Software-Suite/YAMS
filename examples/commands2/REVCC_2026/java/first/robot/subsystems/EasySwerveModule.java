// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Rotations;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.util.CANPorts;
import first.robot.Constants.ModuleConstants;
import first.robot.Constants.NeoMotorConstants;
import org.wpilib.command2.Subsystem;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.SwerveModuleConfig;
import yams.core.mechanisms.swerve.SwerveModule;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * Builds a YAMS {@link SwerveModule} for the REV EasySwerve Module built with NEOs, SPARK MAXs, and
 * a Through Bore Encoder V2 plugged into the steering SPARK MAX.
 *
 * <p>In the REV code this class owned both SPARKs and ran the kinematics math itself. YAMS
 * {@link SwerveModule} already handles state optimization, angle offsets, and odometry positions, so
 * all that is left here is describing the hardware.
 */
public final class EasySwerveModule
{
  // Wheel free speed in wheel rotations per second; used to derive the drive feedforward.
  private static final double kDriveWheelFreeSpeedRps =
      NeoMotorConstants.kFreeSpeed.in(RotationsPerSecond) / ModuleConstants.kDrivingMotorReduction;

  private EasySwerveModule()
  {
    throw new UnsupportedOperationException("This is a utility class!");
  }

  /**
   * Constructs an EasySwerve {@link SwerveModule} and configures the driving and turning motor,
   * encoder, and PID controller.
   *
   * @param subsystem             Subsystem that owns the module.
   * @param name                  Telemetry name of the module.
   * @param location              Module center relative to robot center (+X forward, +Y left).
   * @param drivingCANId          CAN ID of the driving SPARK MAX.
   * @param turningCANId          CAN ID of the turning SPARK MAX.
   * @param chassisAngularOffset  Angle of the module relative to the chassis, in [0, 360) degrees.
   * @param drivingMotorOnBottom  Whether the driving motor is mounted on the bottom of the module.
   * @param turningMotorOnBottom  Whether the turning motor is mounted on the bottom of the module.
   * @return Configured {@link SwerveModule}.
   */
  public static SwerveModule create(Subsystem subsystem, String name, Translation2d location,
                                    int drivingCANId, int turningCANId, Angle chassisAngularOffset,
                                    boolean drivingMotorOnBottom, boolean turningMotorOnBottom)
  {
    SparkMax drivingSpark = new SparkMax(CANPorts.fromBusId(1), drivingCANId, MotorType.kBrushless);
    SparkMax turningSpark = new SparkMax(CANPorts.fromBusId(1), turningCANId, MotorType.kBrushless);

    SmartMotorControllerConfig drivingConfig = new SmartMotorControllerConfig(subsystem)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(ModuleConstants.kDrivingMotorReduction))
        .withWheelDiameter(ModuleConstants.kWheelDiameter)
        // These are example gains you may need to adjust them for your own robot!
        // REV's kP of 0.04 per m/s, re-expressed per wheel rotation per second.
        .withClosedLoopController(0.04 * ModuleConstants.kWheelDiameter.in(Meters) * Math.PI, 0, 0)
        // kV = 12 V / wheel free speed, the same feedforward the REV code computed.
        .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / kDriveWheelFreeSpeedRps))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(60))
        .withMotorInverted(drivingMotorOnBottom)
        .withTelemetry("driveMotor", TelemetryVerbosity.HIGH);

    // The Through Bore Encoder V2 needs these pulse widths set on the SPARK (for V1, set them both
    // to 1.0). YAMS has no setting for them, so they are passed through as the base vendor config.
    SparkMaxConfig turningVendorConfig = new SparkMaxConfig();
    turningVendorConfig.absoluteEncoder
        .startPulseUs(3.88443797)
        .endPulseUs(1.94221899);

    SmartMotorControllerConfig turningConfig = new SmartMotorControllerConfig(subsystem)
        .withVendorConfig(turningVendorConfig)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withGearing(new MechanismGearing(ModuleConstants.kTurningMotorReduction))
        // These are example gains you may need to adjust them for your own robot!
        // REV's kP of 1 per radian, re-expressed per rotation.
        .withClosedLoopController(2 * Math.PI, 0, 0)
        // Close the steering loop on the Through Bore Encoder, which sits on the module output
        // (1:1 with the wheel's azimuth), exactly like the REV code did.
        .withExternalEncoder(turningSpark.getAbsoluteEncoder())
        .withUseExternalFeedbackEncoder(true)
        // Do not invert the turning encoder, since the output shaft rotates in the same
        // direction as the steering motor in the EasySwerve Module.
        .withExternalEncoderInverted(false)
        .withExternalEncoderZeroOffset(chassisAngularOffset)
        // Report the encoder in [-0.5, 0.5) rotations, matching the wrapping range below.
        .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5))
        // Enable PID wrap around for the turning motor. This will allow the PID controller to go
        // through 0 to get to the setpoint i.e. going from 350 degrees to 10 degrees will go
        // through 0 rather than the other direction which is a longer route.
        .withContinuousWrapping(Rotations.of(-0.5), Rotations.of(0.5))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(60))
        .withMotorInverted(!turningMotorOnBottom)
        .withTelemetry("angleMotor", TelemetryVerbosity.HIGH);

    SmartMotorController drivingSMC = new SparkWrapper(drivingSpark, DCMotor.getNEO(1), drivingConfig);
    SmartMotorController turningSMC = new SparkWrapper(turningSpark, DCMotor.getNEO(1), turningConfig);

    SwerveModuleConfig moduleConfig = new SwerveModuleConfig(drivingSMC, turningSMC)
        .withLocation(location)
        // Optimize the reference state to avoid spinning further than 90 degrees.
        .withOptimization(true)
        .withTelemetry(name, TelemetryVerbosity.HIGH);
    return new SwerveModule(moduleConfig);
  }
}
