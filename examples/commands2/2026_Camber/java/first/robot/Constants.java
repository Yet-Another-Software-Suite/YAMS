// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meter;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Seconds;

import com.ctre.phoenix6.CANBus;
import first.robot.utils.FieldConstants.Hub;
import first.robot.utils.FieldConstants.Outpost;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.filter.Debouncer.DebounceType;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Time;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.telemetry.enums.TelemetryVerbosity;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{

  public static class CANIDS
  {

    // Systemcore has no "rio" bus; its first onboard CAN port takes that role. Every device is on it.
    public static final CANPort canPort         = CANPort.CAN_S0;
    public static final CANBus  canBus          = new CANBus(canPort);
    public static final int     shooterCANID    = 4;
    public static final int     shooterCANIDtwo = 41; //whose idea was this :(
    public static final int     indexerCANID    = 3;
  }

  public static class SwerveDrive
  {

    public static final Distance robotWidth  = Inches.of(24);
    public static final Distance robotLength = Inches.of(32);
    public static final double   maxSpeed    = 9.2 * .5; // This still seems somewhat fast.
    public static final Pose2d   startPose   = new Pose2d(new Translation2d(Meter.of(3.5),
                                                                            Meter.of(4)),

                                                          // red 13x 4y
                                                          // blue 4x, 4y
                                                          Rotation2d.fromDegrees(0));
    //red 180
    //blue 0

    public static class Setpoints
    {

      public static final Pose2d robotPoseAtOutpost = new Pose2d(Outpost.centerPoint, Rotation2d.ZERO)
          .plus(new Transform2d(robotLength.div(2).in(Meters), 0, Rotation2d.ZERO));
      public static final Pose2d robotPoseAtHub     = Hub.nearFace
          .plus(new Transform2d(robotLength.div(2).in(Meters), 0, Rotation2d.ZERO));
    }

    /**
     * Swerve module hardware, from the YAGSL JSON files in {@code deploy/swerve}. YAMS builds the
     * drivetrain from these values instead.
     */
    public static class Modules
    {

      /**
       * One module's devices and placement.
       *
       * @param name          Module name; the name of its YAGSL JSON file.
       * @param driveId       Drive SPARK MAX CAN ID.
       * @param angleId       Angle SPARK MAX CAN ID.
       * @param encoderId     CANcoder CAN ID.
       * @param encoderOffset CANcoder reading while the wheel faces forward.
       * @param front         Distance in front of the robot center.
       * @param left          Distance to the left of the robot center.
       */
      public record Module(String name, int driveId, int angleId, int encoderId, Angle encoderOffset,
                           Distance front, Distance left)
      {

      }

      // The locations are copied as-is from the JSON files: "frontleft" is the back left corner and
      // so on. Only the names are swapped; kinematics uses the locations.
      public static final Module frontLeft  = new Module("frontleft", 7, 8, 0, Degrees.of(0.615234375),
                                                         Inches.of(-14.25), Inches.of(13.25));
      public static final Module frontRight = new Module("frontright", 6, 5, 2, Degrees.of(331.875),
                                                         Inches.of(-14.25), Inches.of(-13.25));
      public static final Module backLeft   = new Module("backleft", 12, 11, 3, Degrees.of(36.123046875),
                                                         Inches.of(14.25), Inches.of(13.25));
      public static final Module backRight  = new Module("backright", 2, 1, 1, Degrees.of(279.755859375),
                                                         Inches.of(14.25), Inches.of(-13.25));

      public static final DCMotor  driveMotor        = DCMotor.getNEO(1);
      public static final DCMotor  angleMotor        = DCMotor.getNEO(1);
      public static final double   driveGearRatio    = 5.36;
      public static final double   angleGearRatio    = 18.75;
      public static final Distance wheelDiameter     = Inches.of(4);
      public static final boolean  driveInverted     = true;
      public static final boolean  angleInverted     = true;
      public static final Current  driveCurrentLimit = Amps.of(40);
      public static final Current  angleCurrentLimit = Amps.of(20);
      public static final Time     rampRate          = Seconds.of(0.25);

      // YAGSL gains were duty cycle per m/s (drive) and per degree (angle); YAMS gains are volts per
      // wheel rotation per second and per module rotation, so they are also scaled by 12 V.
      public static final double driveKP = 0.0020645 * wheelDiameter.in(Meters) * Math.PI * 12;
      public static final double angleKP = 0.01 * 360 * 12;
      // Simulation only: the simulated steering lags far behind its setpoint with angleKP.
      public static final double angleSimKP = 30;

      // YAGSL added a drive feedforward of 12 V at the maximum speed, in volts per m/s; here it is
      // per wheel rotation per second.
      public static final SimpleMotorFeedforward driveFeedforward =
          new SimpleMotorFeedforward(0, 12.0 / maxSpeed * wheelDiameter.in(Meters) * Math.PI);

      // Distance from the robot center to the farthest module. YAGSL's maximum angular velocity is the
      // maximum speed divided by this radius.
      public static final Distance driveBaseRadius = Meters.of(
          new Translation2d(frontLeft.front(), frontLeft.left()).getNorm());

      public static final LinearVelocity  maxLinearVelocity  = MetersPerSecond.of(maxSpeed);
      public static final AngularVelocity maxAngularVelocity =
          RadiansPerSecond.of(maxSpeed / driveBaseRadius.in(Meters));

      // controllerproperties.json heading P. YAGSL multiplied the heading PID output by the maximum
      // angular velocity; the YAMS rotation controller outputs rad/s directly.
      public static final double headingKP = 0.2218 * maxAngularVelocity.in(RadiansPerSecond);

      // controllerproperties.json angleJoystickRadiusDeadband: the heading stick only picks a new
      // heading once pushed this far.
      public static final double angleJoystickRadiusDeadband = 0.5;
    }
  }

  public static final TelemetryVerbosity verbosity = TelemetryVerbosity.HIGH;

  public static class Indexer
  {

    public static final SmartMotorControllerConfig smc = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("IndexerMotor", verbosity)
        .withIdleMode(MotorMode.BRAKE)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withGearing(new MechanismGearing(GearBox.fromTeeth(12, 80, 36)))
        .withClosedLoopController(0, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0, 10, 0))
        .withMomentOfInertia(Inches.of(2), Pounds.of(1));

    public static final DCMotor        motor  = DCMotor.getNEO(1);
    public static final FlyWheelConfig config = new FlyWheelConfig()
        .withTelemetry("Indexer", verbosity)
        .withDiameter(Inches.of(2));

    public static class Setpoints
    {

      public static final AngularVelocity indexSpeed = RPM.of(1000);
    }
  }

  public static class Shooter
  {

    public static class Setpoints
    {

      public static final AngularVelocity tolerance           = RPM.of(60); //60
      public static final AngularVelocity lowRPM              = RPM.of(1100); // 3100
      public static final AngularVelocity midRPM              = RPM.of(1500); // 3500
      public static final AngularVelocity high                = RPM.of(1600); // 3600
      public static final AngularVelocity maxRPM              = RPM.of(3600); // 3600
      public static final AngularVelocity autonomousPeriodRPM = RPM.of(3350);

    }

    public static final Debouncer                  flyWheelRecoveryDebouncer = new Debouncer(0.25,
                                                                                             DebounceType.FALLING);
    public static final DCMotor                    motor                     = DCMotor.getKrakenX60(2);
    // The inverted follower (CAN 41) is created by ShooterSubsystem and added with withFollowers.
    public static final SmartMotorControllerConfig smc                       = new SmartMotorControllerConfig()
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withTelemetry("ShooterMotor", verbosity)
        .withIdleMode(MotorMode.COAST)
        .withMotorInverted(false)
        .withStatorCurrentLimit(Amps.of(40))
        .withGearing(new MechanismGearing(GearBox.fromTeeth(40, 60, 60)))
        .withClosedLoopController(0.001, 0, 0)
        .withFeedforward(new SimpleMotorFeedforward(0.000, 0.170, 0.1))
        .withMomentOfInertia(Inches.of(4), Pounds.of(2));
    public static final FlyWheelConfig             config                    = new FlyWheelConfig()
        .withTelemetry("Shooter", verbosity)
        .withDiameter(Inches.of(4));
  }
}
