// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.util.CANPorts;
import first.robot.Constants.NeoMotorConstants;
import first.robot.Constants.ShooterSubsystemConstants;
import first.robot.Constants.ShooterSubsystemConstants.FeederSetpoints;
import first.robot.Constants.ShooterSubsystemConstants.FlywheelSetpoints;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.command2.button.Trigger;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Pair;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.FlyWheel;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.core.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.core.motorcontrollers.local.SparkWrapper;

/**
 * Shooter for the 2026 REV ION Starter Bot. Two Vortexes drive the flywheel (right leader, left
 * follower) and a third Vortex runs the feeder that pushes fuel into the flywheel.
 *
 * <p>The flywheel and the feeder are both YAMS {@link FlyWheel} mechanisms; the feeder is driven
 * open loop but is still a velocity mechanism.
 *
 * <p>For the flywheel, configuring a trapezoidal profile on the
 * {@link SparkWrapper} makes YAMS command the SPARK with MAXMotion Velocity control, which is the
 * same smooth spin-up the REV code set up by hand.
 */
public class ShooterSubsystem extends SubsystemBase
{
  // Initialize flywheel SPARKs. The leader is wrapped by YAMS; the follower is handed to the
  // leader's config and mirrored by the SPARK itself.
  private final SparkFlex flywheelMotor         = new SparkFlex(CANPorts.fromBusId(1),
                                                                ShooterSubsystemConstants.kFlywheelMotorCanId,
                                                                MotorType.kBrushless);
  private final SparkFlex flywheelFollowerMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                                ShooterSubsystemConstants.kFlywheelFollowerMotorCanId,
                                                                MotorType.kBrushless);

  // Initialize feeder SPARK. We will use open loop control for this.
  private final SparkFlex feederMotor = new SparkFlex(CANPorts.fromBusId(1),
                                                      ShooterSubsystemConstants.kFeederMotorCanId,
                                                      MotorType.kBrushless);

  private final SmartMotorControllerConfig flywheelConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.CLOSED_LOOP)
      // The flywheel is direct drive in the REV code (no conversion factor on the encoder).
      .withGearing(new MechanismGearing(1.0))
      // REV's kP of 0.0002 per RPM, re-expressed per rotation per second.
      .withClosedLoopController(0.0002 * 60, 0, 0)
      // kV = 12 V / free speed: at the Vortex free speed the feedforward alone commands 12 V, so
      // kP only has to clean up the residual error.
      .withFeedforward(new SimpleMotorFeedforward(0, 12.0 / NeoMotorConstants.kVortexFreeSpeed.in(RotationsPerSecond)))
      // Same values as the REV MAXMotion config: 5000 RPM cruise, 10000 RPM/s acceleration.
      .withTrapezoidalProfile(RPM.of(5000), RPM.per(Second).of(10000))
      .withMotorInverted(true)
      // COAST so the flywheel spins down naturally after a command ends.
      .withIdleMode(MotorMode.COAST)
      .withClosedLoopRampRate(Seconds.of(1.0))
      .withOpenLoopRampRate(Seconds.of(1.0))
      .withStatorCurrentLimit(Amps.of(80))
      // Simulation only: rough estimate of the flywheel's inertia so spin-up time looks plausible.
      .withMomentOfInertia(Inches.of(2), Pounds.of(1))
      .withTelemetry("FlywheelMotor", TelemetryVerbosity.HIGH)
      // The follower sits on the opposite side of the shooter, so it spins inverted from the leader.
      .withFollowers(Pair.of(flywheelFollowerMotor, true));

  private final SmartMotorControllerConfig feederConfig = (SmartMotorControllerConfig) new SmartMotorControllerConfig(this)
      .withControlMode(ControlMode.OPEN_LOOP)
      .withGearing(new MechanismGearing(1.0))
      .withMotorInverted(true)
      .withIdleMode(MotorMode.COAST)
      .withOpenLoopRampRate(Seconds.of(1.0))
      .withStatorCurrentLimit(Amps.of(60))
      // Simulation only: rough estimate of the feeder rollers' inertia.
      .withMomentOfInertia(Inches.of(1), Pounds.of(0.5))
      .withTelemetry("FeederMotor", TelemetryVerbosity.HIGH);

  private final SmartMotorController flywheelMotorController = new SparkWrapper(flywheelMotor,
                                                                                DCMotor.getNeoVortex(2),
                                                                                flywheelConfig);
  private final SmartMotorController feederMotorController   = new SparkWrapper(feederMotor,
                                                                                DCMotor.getNeoVortex(1),
                                                                                feederConfig);

  private final FlyWheel flywheel = new FlyWheel(new FlyWheelConfig()
                                                     .withDiameter(Inches.of(4))
                                                     .withTelemetry("Flywheel", TelemetryVerbosity.HIGH),
                                                 flywheelMotorController);

  // Roller diameter is an estimate; it only affects telemetry and the simulation display.
  private final FlyWheel feeder = new FlyWheel(new FlyWheelConfig()
                                                   .withDiameter(Inches.of(2))
                                                   .withTelemetry("Feeder", TelemetryVerbosity.HIGH),
                                               feederMotorController);

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem()
  {
    System.out.println("---> ShooterSubsystem initialized");
  }

  /**
   * Trigger: Is the flywheel spinning at the required velocity?
   */
  public final Trigger isFlywheelSpinning = new Trigger(
      () -> flywheel.getSpeed().isNear(FlywheelSetpoints.kShootRpm, FlywheelSetpoints.kVelocityTolerance)
            || flywheel.getSpeed().gt(FlywheelSetpoints.kShootRpm));

  public final Trigger isFlywheelSpinningBackwards = new Trigger(
      () -> flywheel.getSpeed().isNear(FlywheelSetpoints.kShootRpm.unaryMinus(), FlywheelSetpoints.kVelocityTolerance)
            || flywheel.getSpeed().lt(FlywheelSetpoints.kShootRpm.unaryMinus()));

  /**
   * Trigger: Is the flywheel stopped?
   */
  public final Trigger isFlywheelStopped = new Trigger(
      () -> flywheel.getSpeed().isNear(RPM.of(0), FlywheelSetpoints.kVelocityTolerance));

  /**
   * Drive the flywheels to their set velocity. YAMS uses MAXMotion velocity control because a
   * trapezoidal profile is configured, giving a smooth acceleration to the setpoint.
   */
  private void setFlywheelVelocity(AngularVelocity velocity)
  {
    flywheelMotorController.startClosedLoopController();
    flywheelMotorController.setVelocity(velocity);
  }

  /** Stop the flywheel without actively braking it to zero. */
  private void stopFlywheel()
  {
    flywheelMotorController.setDutyCycle(0);
  }

  /** Set the feeder motor power in the range of [-1, 1]. */
  private void setFeederPower(double power)
  {
    feeder.setDutyCycleSetpoint(power);
  }

  /**
   * Command to run the flywheel motors. When the command is interrupted, e.g. the button is
   * released, the motors will stop.
   */
  public Command runFlywheelCommand()
  {
    return this.startEnd(
        () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
        () -> this.setFlywheelVelocity(RPM.of(0))).withName("Spinning Up Flywheel");
  }

  /**
   * Command to run the feeder and flywheel motors. When the command is interrupted, e.g. the button
   * is released, the motors will stop.
   */
  public Command runFeederCommand()
  {
    return this.startEnd(
        () -> {
          this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
          this.setFeederPower(FeederSetpoints.kFeed);
        }, () -> {
          this.setFlywheelVelocity(RPM.of(0));
          this.setFeederPower(0.0);
        }).withName("Feeding");
  }

  /**
   * Meta-command to operate the shooter. The Flywheel starts spinning up and when it reaches the
   * desired speed it starts the Feeder.
   */
  public Command runShooterCommand()
  {
    return this.startEnd(
        () -> this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm),
        this::stopFlywheel
    ).until(isFlywheelSpinning).andThen(
        this.startEnd(
            () -> {
              this.setFlywheelVelocity(FlywheelSetpoints.kShootRpm);
              this.setFeederPower(FeederSetpoints.kFeed);
            }, () -> {
              this.stopFlywheel();
              this.setFeederPower(0.0);
            })
    ).withName("Shooting");
  }

  /** Current flywheel velocity. */
  public AngularVelocity getFlywheelVelocity()
  {
    return flywheel.getSpeed();
  }

  @Override
  public void periodic()
  {
    // Replaces the SmartDashboard output/current/velocity entries from the REV code.
    flywheel.updateTelemetry();
    feeder.updateTelemetry();
  }

  @Override
  public void simulationPeriodic()
  {
    flywheel.simIterate();
    feeder.simIterate();
  }
}
