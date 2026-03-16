package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class TurretSubsystem extends SubsystemBase
{

  double[] ratio = {144 / 15, 5, 1.08};

  SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      .withSimClosedLoopController(0.0, 0.0, 0)
      // 99.0, 0.0, .6
      .withClosedLoopController(0.0, 0.0, 0)

      // Configure Motor and Mechanism properties
      .withGearing(new MechanismGearing(new GearBox(ratio)))
      .withIdleMode(MotorMode.BRAKE)
      .withMotorInverted(false)
      .withFeedforward(new ArmFeedforward(0.5, 0.0, 5.0, 0))
      .withSimFeedforward(new ArmFeedforward(0.5, 0.0, 5.0, 0))

      // 0.0,5.5`
      // Setup Telemetry
      .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
      // Power Optimization
      .withStatorCurrentLimit(Amps.of(60));
  // .withClosedLoopRampRate(Seconds.of(0.0))

  // .withOpenLoopRampRate(Seconds.of(0.0));
  SmartMotorController motor = new TalonFXWrapper(new TalonFX(12),
                                                  DCMotor.getKrakenX60(1),
                                                  motorConfig);

  PivotConfig m_config = new PivotConfig(motor)
      .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
      .withHardLimit(Degrees.of(-360), Degrees.of(360)) // Hard limit bc wiring prevents infinitpe spinning
      // .withSoftLimits(Degrees.of(-360), Degrees.of(360))
      .withTelemetry("Turret", TelemetryVerbosity.HIGH) // Telemetry
      .withMOI(yams.units.YUnits.PoundSquareInches.of(0.01)); // MOI Calculation

  private Pivot turret = new Pivot(m_config);


  // Robot to turret transform, from center of robot to turret.
  private final Transform3d roboToTurret = new Transform3d(Feet.of(-1.5), Feet.of(0), Feet.of(0.5), Rotation3d.kZero);

  public TurretSubsystem()
  {
    // TODO: Set the default command, if any, for this subsystem by calling setDefaultCommand(command)
    //       in the constructor or in the robot coordination class, such as RobotContainer.
    //       Also, you can call addChild(name, sendableChild) to associate sendables with the subsystem
    //       such as SpeedControllers, Encoders, DigitalInputs, etc.
  }

  public Pose2d getPose(Pose2d robotPose)
  {
    return robotPose.plus(new Transform2d(
        roboToTurret.getTranslation().toTranslation2d(), roboToTurret.getRotation().toRotation2d()));
  }

 public ChassisSpeeds getVelocity(ChassisSpeeds robotVelocity, Angle robotAngle)
  {

      Translation2d rRobot = roboToTurret.getTranslation().toTranslation2d(); // in robot frame
    Translation2d rWorld = rRobot.rotateBy(Rotation2d.fromRadians(robotAngle.in(Radians))); // rotate into field frame

      double omega = robotVelocity.omegaRadiansPerSecond; // robot yaw rate (rad/s)

      // rotational linear velocity at turret (v_rot = ω × r_world)
      double vRotX = -omega * rWorld.getY();
      double vRotY =  omega * rWorld.getX();

      // final turret linear velocity in field frame
      double turretVx = robotVelocity.vxMetersPerSecond + vRotX;
      double turretVy = robotVelocity.vyMetersPerSecond + vRotY;

      // turret angular velocity in field frame
      double turretOmega = omega + motor.getMechanismVelocity().in(RadiansPerSecond);

      return new ChassisSpeeds(turretVx, turretVy, turretOmega);

  }

  public void periodic()
  {
    turret.updateTelemetry();
  }

  public void simulationPeriodic()
  {
    turret.simIterate();
  }

  public Command turretCmd(double dutycycle)
  {
    return turret.set(dutycycle);
  }

  public Command sysId()
  {
    return turret.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
  }

  public Command setAngle(Angle angle)
  {
    return turret.setAngle(angle);
  }

  public void setAngleSetpoint(Angle measure)
  {
    turret.setMechanismPositionSetpoint(measure);
  }
}
