// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.local;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Celsius;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkAbsoluteEncoderSim;
import com.revrobotics.sim.SparkRelativeEncoderSim;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.ControlType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkSim;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.function.Supplier;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.ExponentialProfile;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Notifier;
import org.wpilib.system.Timer;
import org.wpilib.units.AngularAccelerationUnit;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Force;
import org.wpilib.units.measure.LinearAcceleration;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Velocity;
import org.wpilib.units.measure.Voltage;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import org.wpilib.util.Pair;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.MechanismGearing;
import yams.core.math.DerivativeTimeFilter;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.simulation.BatterySim;
import yams.core.motorcontrollers.simulation.DCMotorSimSupplier;
import yams.core.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.core.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Spark wrapper for REV Spark Motor controllers.
 *
 * <p><b>External encoder discontinuity point support (SparkAbsoluteEncoder):</b>
 *
 * <ul>
 * <li>{@code 0.5} rotations sensor range is [-0.5, 0.5), {@code zeroCentered = true}
 * <li>{@code 1.0} rotations sensor range is [0, 1), {@code zeroCentered = false}
 * </ul>
 *
 * A discontinuity point <b>must</b> be configured via {@link yams.core.motorcontrollers.SmartMotorControllerConfig#withExternalEncoderDiscontinuityPoint}
 * whenever a {@link com.revrobotics.spark.SparkAbsoluteEncoder} is used as the external encoder.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // Configure and create a SPARK MAX (NEO motor) on CAN ID 3
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withMotorInverted(false)
 *     .withStatorCurrentLimit(Amps.of(40))
 *     .withClosedLoopController(0.2,0,0)
 *     .withExternalEncoderDiscontinuityPoint(Rotations.of(0.5)); // required for
 * SparkAbsoluteEncoder SmartMotorController motor = new SparkWrapper( new SparkMax(3,
 * MotorType.kBrushless), DCMotor.getNEO(1), config); Arm arm = new Arm(new
 * ArmConfig(motor).withLength(Meters.of(0.6)));
 * }</pre>
 */
public class SparkWrapper extends SmartMotorController {
  /** Spark motor controller */
  private final SparkBase m_spark;
  /** Spark Closed loop controller. */
  private final SparkClosedLoopController m_sparkPidController;
  /** Motor type. */
  private final DCMotor m_motor;
  /** Spark base configuration. */
  private final SparkBaseConfig m_sparkBaseConfig;
  /** Spark relative encoder. */
  private final RelativeEncoder m_sparkRelativeEncoder;
  /** Spark relative encoder sim object. */
  private Optional<SparkRelativeEncoderSim> sparkRelativeEncoderSim = Optional.empty();
  /** Spark simulation. */
  private Optional<SparkSim> sparkSim = Optional.empty();
  /** Spark absolute encoder. */
  private Optional<AbsoluteEncoder> m_sparkAbsoluteEncoder = Optional.empty();
  /** Spark absolute encoder sim object */
  private Optional<SparkAbsoluteEncoderSim> m_sparkAbsoluteEncoderSim = Optional.empty();
  /** DC Motor Sim. */
  private Optional<DCMotorSim> m_dcMotorSim = Optional.empty();
  /** REV Control type to use for position control. */
  private ControlType m_positionControlType = ControlType.kPosition;
  /** REV Control type to use for velocity control. */
  private ControlType m_velocityControlType = ControlType.kVelocity;
  /** REV Closed loop slot. */
  private ClosedLoopSlot m_closedLoopSlot = ClosedLoopSlot.kSlot0;
  /** Acceleration filter. */
  private DerivativeTimeFilter m_accelerationFilter = new DerivativeTimeFilter(Milliseconds.of(20));
  /**
   * Alert shown when external encoder gearing is set alongside an external encoder discontinuity
   * point.
   */
  private Alert m_externalEncoderGearingDiscontinuityAlert;

  /**
   * Create a {@link SmartMotorController} from {@link SparkMax} or {@link SparkFlex}
   *
   * @param controller {@link SparkMax} or {@link SparkFlex}
   * @param motor      {@link DCMotor} controller by the {@link SparkFlex} or {@link SparkMax}. Must
   *                   be a
   *                   brushless motor.
   * @param config     {@link SmartMotorControllerConfig} to apply.
   * @throws SmartMotorControllerConfigurationException if the vendor config does not match the
   *                                                    controller type (a SparkMaxConfig is
   *                                                    required for a SparkMax and a
   *                                                    SparkFlexConfig for a SparkFlex), if the
   *                                                    motor is a NEO 550 with no stator current
   *                                                    limit or a stator current limit above 40A,
   *                                                    or if
   *                                                    {@link #applyConfig(SmartMotorControllerConfig)}
   *                                                    rejects the config.
   * @throws IllegalArgumentException if the controller is neither a SparkMax nor a SparkFlex, or if
   *                                  {@link #applyConfig(SmartMotorControllerConfig)} rejects the
   *                                  config.
   */
  public SparkWrapper(SparkBase controller, DCMotor motor, SmartMotorControllerConfig<?> config) {
    if (controller instanceof SparkMax) {
      if (config.getVendorConfig().isPresent()) {
        var genCfg = config.getVendorConfig().get();
        if (!(genCfg instanceof SparkMaxConfig)) {
          throw new SmartMotorControllerConfigurationException("SparkMaxConfig is the only acceptable vendor config for SparkMax controllers.", "SparkMaxConfig not found.", ".withVendorConfig(new SparkMaxConfig())");
        }
        m_sparkBaseConfig = (SparkMaxConfig) genCfg;
      } else {
        m_sparkBaseConfig = new SparkMaxConfig();
      }
    } else if (controller instanceof SparkFlex) {
      if (config.getVendorConfig().isPresent()) {
        var genCfg = config.getVendorConfig().get();
        if (!(genCfg instanceof SparkFlexConfig)) {
          throw new SmartMotorControllerConfigurationException("SparkFlexConfig is the only acceptable vendor config for SparkFlex controllers.", "SparkFlexConfig not found.", ".withVendorConfig(new SparkFlexConfig())");
        }
        m_sparkBaseConfig = (SparkFlexConfig) genCfg;
      } else {
        m_sparkBaseConfig = new SparkFlexConfig();
      }
    } else {
      throw new IllegalArgumentException("[ERROR] Unsupported controller type: " + controller.getClass().getSimpleName());
    }

    this.m_motor = motor;
    m_spark = controller;
    m_sparkPidController = m_spark.getClosedLoopController();
    this.m_config = config;
    m_systemCoreClosedLoopAlert = Optional.of(new Alert("YAMS", buildAlertId("Spark", m_spark.getDeviceId(), "ClosedLoop"), getName() + " closed loop controller is running on the RIO.", Alert.Level.MEDIUM));
    m_externalEncoderGearingDiscontinuityAlert = new Alert("YAMS", buildAlertId("Spark", m_spark.getDeviceId(), "ExternalEncoderGearingDiscontinuity"),
        getName() + (" external encoder gearing set while ExternalEncoderDiscontinuityPoint is also set; " + "the discontinuity point will NOT be moved by the gearing, wrapping will occur " + "non-uniformly"), Level.HIGH);
    m_sparkRelativeEncoder = controller.getEncoder();
    setupSimulation();
    applyConfig(config);
    checkConfigSafety();
  }

  /**
   * Run the configuration until it succeeds or times out.
   *
   * @param config Lambda supplier returning the error state.
   * @return Successful configuration
   */
  private boolean configureSpark(Supplier<REVLibError> config) {
    for (int i = 0; i < 8; i++) {
      if (config.get() == REVLibError.kOk) {
        return true;
      }
      Timer.delay(Milliseconds.of(1));
    }
    return false;
  }

  @Override
  public void setupSimulation() {
    if (RobotBase.isSimulation()) {
      var setupRan = sparkSim.isPresent();
      if (!setupRan) {
        sparkSim = Optional.of(new SparkSim(m_spark, m_motor));
        sparkRelativeEncoderSim = Optional.of(sparkSim.get().getRelativeEncoderSim());
        m_dcMotorSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(m_motor, m_config.getMOI(), m_config.getGearing().getMechanismToRotorRatio()), m_motor));
        setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
      }
      m_config.getStartingPosition().ifPresent(startingPos -> {
        sparkSim.get().setPosition(startingPos.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations));
        sparkRelativeEncoderSim.get().setPosition(startingPos.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations));
        m_simSupplier.ifPresent(sim -> sim.setMechanismPosition(startingPos));
      });
    }
  }

  @Override
  public void seedRelativeEncoder() {
    if (m_sparkAbsoluteEncoder.isPresent()) {
      var relativeRotFromAbsRot = m_sparkAbsoluteEncoder.get().getPosition().get() * m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio() * m_config.getGearing().getMechanismToRotorRatio();
      m_sparkRelativeEncoder.setPosition(relativeRotFromAbsRot);
      sparkRelativeEncoderSim.ifPresent(sparkRelativeEncoderSim -> sparkRelativeEncoderSim.setPosition(relativeRotFromAbsRot));
    }
  }

  @Override
  public void synchronizeRelativeEncoder() {
    if (m_config.getFeedbackSynchronizationThreshold().isPresent()) {
      if (m_sparkAbsoluteEncoder.isPresent()) {
        if (!Rotations.of(m_sparkRelativeEncoder.getPosition().get().floatValue() * m_config.getGearing().getRotorToMechanismRatio()).isNear(Rotations.of(m_sparkAbsoluteEncoder.get().getPosition().get().floatValue() * m_config
            .getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio()), m_config.getFeedbackSynchronizationThreshold().get())) {
          seedRelativeEncoder();
        }
      }
    }
  }

  @Override
  public void simIterate() {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      if (!m_simSupplier.get().getUpdatedSim()) {
        m_simSupplier.get().updateSimState();
        m_simSupplier.get().starveUpdateSim();
        BatterySim.calculateVoltage(m_batterySimUUID, m_simSupplier.get().getSupplyCurrent());
      }
      Time simLoop = m_config.getSimulationPeriod();
      m_simSupplier.ifPresent(mSimSupplier -> {
        // iterate() expects RPM here; may need revisiting once conversion factors are added back.
        sparkSim.ifPresent(sim -> sim.iterate(mSimSupplier.getMechanismVelocity().times(m_config.getGearing().getMechanismToRotorRatio()).in(RPM), mSimSupplier.getMechanismSupplyVoltage().in(Volts), simLoop.in(Second)));
        sparkRelativeEncoderSim.ifPresent(sim -> sim.iterate(mSimSupplier.getMechanismVelocity().times(m_config.getGearing().getMechanismToRotorRatio()).in(RPM), simLoop.in(Seconds)));
        m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.iterate(mSimSupplier.getMechanismVelocity().times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()).in(RPM), simLoop.in(
            Seconds)));
      });
      // TODO: Uncomment after the 2026 season
      //      m_looseFollowers.ifPresent(smcs -> {for(var f : smcs){f.simIterate();}});
    }
  }

  @Override
  public void setIdleMode(MotorMode mode) {
    m_sparkBaseConfig.idleMode(mode == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    configureSpark(() -> m_spark.configure(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   * @throws UnsupportedOperationException if called outside of simulation, since REV Sparks do not
   *                                       support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(LinearVelocity velocity) {
    setEncoderVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    if (m_sparkAbsoluteEncoder.isPresent()) {
      m_sparkBaseConfig.absoluteEncoder.zeroOffset(getMechanismPosition().minus(angle.times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio())).in(Rotations));
      m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setPosition(angle.times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()).in(Rotations)));
    }
    var rotor = angle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations);
    m_sparkRelativeEncoder.setPosition(rotor);
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setPosition(rotor));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException if called outside of simulation, since REV Sparks do not
   *                                       support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(AngularVelocity velocity) {
    if (!RobotBase.isSimulation())
      throw new UnsupportedOperationException("REV Spark does not support setting encoder velocity.");
    sparkRelativeEncoderSim.ifPresent(relativeEncoderSim -> relativeEncoderSim.setVelocity(velocity.in(RotationsPerSecond)));
    m_sparkAbsoluteEncoderSim.ifPresent(absoluteEncoderSim -> absoluteEncoderSim.setVelocity(velocity.in(RotationsPerSecond)));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void setEncoderPosition(Distance distance) {
    setEncoderPosition(m_config.convertToMechanism(distance));
  }

  @Override
  public void setPosition(Angle angle) {
    setpointVelocity = Optional.empty();
    setpointFeedforwardForce = Optional.empty();
    setpointPosition = Optional.ofNullable(angle);
    if (m_expoProfile.isEmpty() && m_lqr.isEmpty() && angle != null) {
      configureSpark(() -> m_sparkPidController.setSetpoint(angle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations), m_positionControlType, m_closedLoopSlot));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setPosition(angle);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void setPosition(Distance distance) {
    setPosition(m_config.convertToMechanism(distance));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void setVelocity(LinearVelocity velocity) {
    setVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity) {
    setpointPosition = Optional.empty();
    setpointVelocity = Optional.ofNullable(angularVelocity);
    setpointFeedforwardForce = Optional.empty();
    if (m_lqr.isEmpty() && angularVelocity != null) {
      configureSpark(() -> m_sparkPidController.setSetpoint(setpointVelocity.orElse(RPM.of(0)).times(m_config.getGearing().getMechanismToRotorRatio()).in(RotationsPerSecond), m_velocityControlType, m_closedLoopSlot));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setVelocity(angularVelocity);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if a feedforward force is given without an
   *                                                    LQR controller and the mechanism
   *                                                    circumference is not configured.
   */
  @Override
  public void setVelocity(AngularVelocity angularVelocity, Force feedforwardForce) {
    setpointPosition = Optional.empty();
    setpointVelocity = Optional.ofNullable(angularVelocity);
    setpointFeedforwardForce = Optional.ofNullable(feedforwardForce);
    if (m_lqr.isEmpty() && angularVelocity != null && setpointFeedforwardForce.isPresent()) {
      Voltage feedforwardVoltage = m_config.convertToVoltage(getDCMotor(), angularVelocity, feedforwardForce);
      configureSpark(() -> m_sparkPidController.setSetpoint(setpointVelocity.orElse(RPM.of(0)).times(m_config.getGearing().getMechanismToRotorRatio()).in(RotationsPerSecond), m_velocityControlType, m_closedLoopSlot, feedforwardVoltage.in(Volts),
          ArbFFUnits.kVoltage));
      m_looseFollowers.ifPresent(smcs -> {
        for (var f : smcs) {
          f.setVelocity(angularVelocity, feedforwardForce);
        }
      });
    } else {
      setVelocity(angularVelocity);
    }
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if a supply current limit is configured; if
   *                                                    an external encoder discontinuity point,
   *                                                    zero offset, inversion, or gearing is
   *                                                    configured without an external encoder; if a
   *                                                    closed loop control period, closed loop
   *                                                    controller maximum voltage, or feedback
   *                                                    synchronization threshold is configured
   *                                                    without an exponential profile or LQR
   *                                                    controller; if a temperature cutoff is
   *                                                    configured without an exponential or
   *                                                    trapezoidal profile; if a vendor control
   *                                                    request is configured; if the continuous
   *                                                    wrapping bounds do not span exactly one
   *                                                    rotation; if a linear closed loop controller
   *                                                    is in use without a mechanism circumference;
   *                                                    or if a required config option is left
   *                                                    unhandled during validation.
   * @throws IllegalArgumentException if a closed loop control period is configured with an
   *                                  exponential profile or LQR controller while not in closed loop
   *                                  mode, if a closed loop tolerance is configured with an LQR
   *                                  controller, if the external encoder is not a
   *                                  SparkAbsoluteEncoder, if a follower is not a SparkMax or
   *                                  SparkFlex, or if relative encoder inversion is configured.
   */
  @Override
  public boolean applyConfig(SmartMotorControllerConfig<?> config) {
    m_config = config;
    config.resetValidationCheck();
    m_systemCoreClosedLoopAlert.ifPresent(alert -> alert.set(false));
    m_externalEncoderGearingDiscontinuityAlert.set(false);
    var mechToRotorRatio = config.getGearing().getMechanismToRotorRatio();

    for (int i = 0; i < 4; i++) {
      if (isMotor(m_motor, DCMotor.getMinion(i))) {
        m_sparkBaseConfig.advanceCommutation(120);
      }
    }
    if (m_spark.isFollower().get()) {
      m_spark.pauseFollowerMode();
      m_sparkBaseConfig.disableFollowerMode();
    }
    m_lqr = config.getLQRClosedLoopController();
    m_pid = config.getPID(m_slot);
    m_looseFollowers = config.getLooselyCoupledFollowers();

    // Handle motion profile
    m_config.getExponentialProfile().ifPresent(expProfile -> {
      m_expoProfile = Optional.of(new ExponentialProfile(expProfile));
    });
    m_config.getTrapezoidProfile().ifPresent(trapProfile -> {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(trapProfile));
      m_sparkBaseConfig.closedLoop.maxMotion.positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
      if (m_config.getLinearClosedLoopControllerUse()) {
        m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(m_config.convertToMechanism(MetersPerSecond.of(trapProfile.maxVelocity)).in(RPM)).maxAcceleration(m_config.convertToMechanism(MetersPerSecondPerSecond.of(trapProfile.maxAcceleration)).in(
            RPM.per(Second)));
      } else {
        m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(RotationsPerSecond.of(trapProfile.maxVelocity).in(RPM)).maxAcceleration(RotationsPerSecondPerSecond.of(trapProfile.maxAcceleration).in(RPM.per(Second)));
      }
      m_positionControlType = ControlType.kMAXMotionPositionControl;
      m_velocityControlType = ControlType.kMAXMotionVelocityControl;
    });

    // Handle closed loop controller thread
    if (m_expoProfile.isPresent() || m_lqr.isPresent()) {
      m_systemCoreClosedLoopAlert.ifPresent(alert -> alert.set(true));
      iterateClosedLoopController();

      if (m_closedLoopControllerThread == null) {
        m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
      } else {
        stopClosedLoopController();
        m_closedLoopControllerThread.stop();
        m_closedLoopControllerThread.close();
        m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
      }

      if (config.getTelemetryName().isPresent()) {
        m_closedLoopControllerThread.setName(config.getTelemetryName().get());
      }
      if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP) {
        startClosedLoopController();
      } else {
        m_closedLoopControllerThread.stop();
        if (config.getClosedLoopControlPeriod().isPresent()) {
          throw new IllegalArgumentException("[Error] Closed loop control period is only supported in closed loop mode.");
        }
      }
    }

    // Set base config options
    config.getOpenLoopRampRate().ifPresent(rate -> m_sparkBaseConfig.openLoopRampRate(rate.in(Seconds)));
    config.getClosedLoopRampRate().ifPresent(rate -> m_sparkBaseConfig.closedLoopRampRate(rate.in(Seconds)));
    config.getMotorInverted().ifPresent(m_sparkBaseConfig::inverted);

    // Control mode is ignored
    config.getMotorControllerMode();

    // Set PID
    m_sparkBaseConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    for (var closedLoopControlSlot : ClosedLoopControllerSlot.values()) {
      var sparkSlot = getSparkClosedLoopSlot(closedLoopControlSlot);
      config.getPID(closedLoopControlSlot).ifPresent(pidController -> {
        m_sparkBaseConfig.closedLoop.pid(pidController.getP(), pidController.getI(), pidController.getD(), sparkSlot);
      });

      // Set feedforward values
      config.getArmFeedforward(closedLoopControlSlot).ifPresent(ff -> {
        m_sparkBaseConfig.closedLoop.feedForward.kS(ff.getKs(), sparkSlot).kV(ff.getKv(), sparkSlot).kA(ff.getKa(), sparkSlot).kCos(ff.getKg(), sparkSlot);
      });
      config.getElevatorFeedforward(closedLoopControlSlot).ifPresent(ff -> {
        m_sparkBaseConfig.closedLoop.feedForward.kS(ff.getKs(), sparkSlot).kV(ff.getKv(), sparkSlot).kA(ff.getKa(), sparkSlot).kG(ff.getKg(), sparkSlot);
      });
      config.getSimpleFeedforward(closedLoopControlSlot).ifPresent(ff -> {
        m_sparkBaseConfig.closedLoop.feedForward.kS(ff.getKs(), sparkSlot).kV(ff.getKv(), sparkSlot).kA(ff.getKa(), sparkSlot);
      });
    }

    // LQR doesn't handle tolerances
    if (m_lqr.isPresent() && config.getClosedLoopTolerance().isPresent()) {
      throw new IllegalArgumentException("[Error] Closed loop tolerance is not supported in LQR mode.");
    }

    // Set closed loop tolerance and profile tolerance to the same thing.
    config.getClosedLoopTolerance().ifPresent(tolerance -> {
      m_sparkBaseConfig.closedLoop.allowedClosedLoopError(tolerance.in(Rotations), m_closedLoopSlot);
      m_sparkBaseConfig.closedLoop.maxMotion.allowedProfileError(tolerance.in(Rotations), m_closedLoopSlot);
      if (config.getLinearClosedLoopControllerUse()) {
        m_pid.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(tolerance).in(Meters)));
      } else {
        m_pid.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
      }
    });

    // Set Mechanism Limits
    config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_sparkBaseConfig.softLimit.reverseSoftLimit(lowerLimit.in(Rotations)).reverseSoftLimitEnabled(config.getMotorControllerMode() == ControlMode.CLOSED_LOOP);
    });
    config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_sparkBaseConfig.softLimit.forwardSoftLimit(upperLimit.in(Rotations)).forwardSoftLimitEnabled(config.getMotorControllerMode() == ControlMode.CLOSED_LOOP);
    });

    // Throw warning about supply stator limits on Spark's
    if (config.getSupplyStallCurrentLimit().isPresent()) {
      throw new SmartMotorControllerConfigurationException("Supply current limits are not supported on Sparks", "Supply current limit not set", "withStatorCurrentLimit");
    }
    // Handle stator current limit.
    if (config.getStatorStallCurrentLimit().isPresent()) {
      m_sparkBaseConfig.smartCurrentLimit(config.getStatorStallCurrentLimit().getAsInt());
    }
    // Handle voltage compensation.
    if (config.getVoltageCompensation().isPresent()) {
      m_sparkBaseConfig.voltageCompensation(config.getVoltageCompensation().get().in(Volts));
    }
    // Setup idle mode.
    if (config.getIdleMode().isPresent()) {
      m_sparkBaseConfig.idleMode(config.getIdleMode().get() == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
    }
    // Setup starting position
    if (config.getStartingPosition().isPresent()) {
      m_sparkRelativeEncoder.setPosition(config.getStartingPosition().get().times(mechToRotorRatio).in(Rotations));
    }
    // PID Wrapping
    if (config.getContinuousWrapping().isPresent() || config.getContinuousWrappingMin().isPresent()) {
      // TODO: Continuous wrapping no longer has bounds, double check bounds and throw an error when
      // unexpected bound shows up
      m_sparkBaseConfig.closedLoop.positionWrappingEnabled(true);
    }

    // Setup external encoder.
    boolean useExternalEncoder = config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent()) {
      var mechToEncoder = config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio();
      Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder instanceof SparkAbsoluteEncoder) {
        m_sparkAbsoluteEncoder = Optional.of((SparkAbsoluteEncoder) externalEncoder);
        var absEncoder = ((SparkAbsoluteEncoder) externalEncoder);

        config.getExternalEncoderInverted().ifPresent(m_sparkBaseConfig.absoluteEncoder::inverted);

        // Set the absolute encoder as the primary feedback sensor for closed loop control.
        if (useExternalEncoder) {
          m_sparkBaseConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
        }

        if (config.getExternalEncoderZeroOffset().isPresent()) {
          m_sparkBaseConfig.absoluteEncoder.zeroOffset(config.getExternalEncoderZeroOffset().get().times(mechToEncoder).in(Rotations));
        }

        if (config.getExternalEncoderDiscontinuityPoint().isPresent()) {
          if (config.getExternalEncoderGearing().isPresent()) {
            m_externalEncoderGearingDiscontinuityAlert.set(true);
          }
          m_sparkBaseConfig.absoluteEncoder.rangeOffset(config.getExternalEncoderDiscontinuityPoint().get().in(Rotations));
        }

        if (RobotBase.isSimulation()) {
          if (m_spark instanceof SparkMax) {
            m_sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkMax) m_spark));
          } else if (m_spark instanceof SparkFlex) {
            m_sparkAbsoluteEncoderSim = Optional.of(new SparkAbsoluteEncoderSim((SparkFlex) m_spark));
          }
          if (config.getStartingPosition().isPresent()) {
            m_sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setPosition(config.getStartingPosition().get().times(mechToEncoder).in(Rotations)));
          }
          if (config.getExternalEncoderZeroOffset().isPresent()) {
            m_sparkAbsoluteEncoderSim.ifPresent(enc -> enc.setZeroOffset(config.getExternalEncoderZeroOffset().get().times(mechToEncoder).in(Rotations)));
          }
        }
      } else {
        throw new IllegalArgumentException("[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName());
      }

      // Set starting position if external encoder is empty.
      if (config.getStartingPosition().isEmpty()) {
        m_sparkRelativeEncoder.setPosition(m_sparkAbsoluteEncoder.get().getPosition().get() * mechToRotorRatio);
      }

    } else {
      if (config.getExternalEncoderDiscontinuityPoint().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder zero center is only available for external encoders", "External encoder zero center could not be applied", ".withExternalEncoderZeroCenter");
      }
      if (config.getExternalEncoderZeroOffset().isPresent()) {
        throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders", "Zero offset could not be applied", ".withExternalEncoderZeroOffset");
      }

      if (config.getExternalEncoderInverted().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder cannot be inverted because no external encoder exists", "External encoder could not be inverted", "withExternalEncoderInverted");
      }

      if (config.getExternalEncoderGearing().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder gearing is not supported when there is no external encoder", "External encoder gearing could not be set", "withExternalEncoderGearing");
      }
    }

    // Configure follower motors
    if (config.getFollowers().isPresent()) {
      for (Pair<Object, Boolean> follower : config.getFollowers().get()) {
        if (follower.getFirst() instanceof SparkMax) {
          var f_cfg = new SparkMaxConfig().follow(m_spark, follower.getSecond());
          m_config.getIdleMode().ifPresent(mode -> f_cfg.idleMode(mode == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast));
          ((SparkMax) follower.getFirst()).configure(f_cfg, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);

        } else if (follower.getFirst() instanceof SparkFlex) {
          var f_cfg = new SparkFlexConfig().follow(m_spark, follower.getSecond());
          m_config.getIdleMode().ifPresent(mode -> f_cfg.idleMode(mode == MotorMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast));
          ((SparkFlex) follower.getFirst()).configure(f_cfg, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);

        } else {
          throw new IllegalArgumentException("[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    if (config.getExternalEncoderZeroOffset().isPresent() && config.getExternalEncoder().isEmpty() && !useExternalEncoder) {
      throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders", "Zero offset could not be applied", ".withZeroOffset");
    }

    if (config.getExternalEncoderInverted().isPresent() && config.getExternalEncoder().isEmpty() && !useExternalEncoder) {
      throw new SmartMotorControllerConfigurationException("External encoder cannot be inverted because no external encoder exists", "External encoder could not be inverted", "withExternalEncoderInverted");
    }

    if (config.getExternalEncoderGearing().isPresent() && config.getExternalEncoder().isEmpty() && !useExternalEncoder) {
      throw new SmartMotorControllerConfigurationException("External encoder gearing is not supported when there is no external encoder", "External encoder gearing could not be set", "withExternalEncoderGearing");
    }

    if (config.getClosedLoopControlPeriod().isPresent() && m_expoProfile.isEmpty() && m_lqr.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Closed loop control period is unsupported without Exponential Profiles", "Closed loop control period does not take affect", ".withClosedLoopControlPeriod");
    }

    if (config.getClosedLoopControllerMaximumVoltage().isPresent() && m_expoProfile.isEmpty() && m_lqr.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Closed loop controller maximum voltage is only available for Exponential Profiled " + "closed loop controllers", "Closed loop controller maximum voltage could not be applied",
          "withClosedLoopControllerMaximumVoltage");
    }

    if (config.getTemperatureCutoff().isPresent() && m_expoProfile.isEmpty() && m_trapezoidProfile.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Temperature cutoff is only available for exponentially profiled closed loop controllers", "Temperature cutoff could not be applied", "withTemperatureCutoff");
    }

    if (config.getFeedbackSynchronizationThreshold().isPresent() && m_expoProfile.isEmpty() && m_lqr.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Feedback synchronization threshold is only available for exponentially profiled closed " + "loop controllers", "Feedback synchronization threshold could not be applied",
          "withFeedbackSynchronizationThreshold");
    }

    if (config.getEncoderInverted().isPresent()) {
      throw new IllegalArgumentException("[ERROR] Spark relative encoder cannot be inverted!");
    }

    if (config.getVendorControlRequest().isPresent()) {
      throw new SmartMotorControllerConfigurationException("Spark(" + m_spark.getDeviceId() + ") does not support the custom control requests!", "Cannot use given control request", "withVendorControlRequest()");
    }

    var resetMode = m_config.getResetPreviousConfig() ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
    config.validateBasicOptions();
    config.validateExternalEncoderOptions();
    return configureSpark(() -> m_spark.configure(m_sparkBaseConfig, resetMode, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters));
  }

  @Override
  public double getDutyCycle() {
    return m_spark.getAppliedOutput().get();
    /* m_simSupplier.map(simSupplier -> simSupplier.getMechanismStatorVoltage().in(Volts) /
                                            simSupplier.getMechanismSupplyVoltage().in(Volts))
                        .orElseGet(spark::getAppliedOutput);*/
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    m_spark.setThrottle(dutyCycle);
    if (dutyCycle == 0.0) {
      m_looseFollowers.ifPresent(looseFollower -> {
        for (var follower : looseFollower) {
          follower.setDutyCycle(dutyCycle);
        }
      });
    }
    //    m_simSupplier.ifPresent(simSupplier ->
    //    simSupplier.setMechanismStatorDutyCycle(dutyCycle));
  }

  @Override
  @Deprecated
  public Optional<Current> getSupplyCurrent() {
    return Optional.empty();
    //    DriverStation.reportError("[WARNING] Supply currently not supported on Spark", true);
    //    return null;
  }

  @Override
  public Current getStatorCurrent() {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getStatorCurrent() : Amps.of(m_spark.getOutputCurrent().get());
  }

  @Override
  public Voltage getVoltage() {
    return m_simSupplier.isPresent() ? m_simSupplier.get().getMechanismStatorVoltage() : Volts.of(m_spark.getAppliedOutput().get() * m_spark.getBusVoltage().get());
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_spark.setVoltage(voltage);
    //    if (voltage.in(Volts) == 0.0)
    //    {m_looseFollowers.ifPresent(looseFollower -> {for (var follower : looseFollower)
    //    {follower.setVoltage(voltage);}});}
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismStatorVoltage(voltage));
  }

  @Override
  public DCMotor getDCMotor() {
    return m_motor;
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public LinearVelocity getMeasurementVelocity() {
    return m_config.convertFromMechanism(getMechanismVelocity());
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public Distance getMeasurementPosition() {
    return m_config.convertFromMechanism(getMechanismPosition());
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public LinearAcceleration getMeasurementAcceleration() {
    return m_config.convertFromMechanism(getMechanismAcceleration());
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    if (m_sparkAbsoluteEncoder.isPresent() && m_config.getUseExternalFeedback()) {
      return RPM.of(m_sparkAbsoluteEncoder.get().getVelocity().get()).times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio());
    }
    return RPM.of(sparkSim.map(SparkSim::getVelocity).orElseGet(() -> m_sparkRelativeEncoder.getVelocity().get())).times(m_config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public AngularAcceleration getMechanismAcceleration() {
    return RotationsPerSecond.per(Microsecond).of(m_accelerationFilter.derivative(getMechanismVelocity().in(RotationsPerSecond)));
  }

  @Override
  public Angle getMechanismPosition() {
    Angle pos = Rotations.of(m_sparkRelativeEncoder.getPosition().get()).times(m_config.getGearing().getRotorToMechanismRatio());
    if (m_sparkAbsoluteEncoder.isPresent() && m_config.getUseExternalFeedback()) {
      pos = Rotations.of(m_sparkAbsoluteEncoder.get().getPosition().get()).times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio());
    }
    return pos;
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return getMechanismVelocity().times(m_config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Angle getRotorPosition() {
    return getMechanismPosition().times(m_config.getGearing().getMechanismToRotorRatio());
  }

  @Override
  public Optional<Angle> getExternalEncoderPosition() {
    return m_sparkAbsoluteEncoder.map(absoluteEncoder -> Rotations.of(absoluteEncoder.getPosition().get()).times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio()));
  }

  @Override
  public Optional<AngularVelocity> getExternalEncoderVelocity() {
    return m_sparkAbsoluteEncoder.map(absoluteEncoder -> RPM.of(absoluteEncoder.getVelocity().get()).times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio()));
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    m_config.withMotorInverted(inverted);
    m_sparkBaseConfig.inverted(inverted);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
  }

  @Override
  public void setEncoderInverted(boolean inverted) {
    m_config.withEncoderInverted(inverted);
    //    if (sparkAbsoluteEncoder.isPresent())
    //    {
    //      sparkBaseConfig.absoluteEncoder.inverted(inverted);
    //    }
    //    sparkBaseConfig.analogSensor.inverted(inverted);
    m_sparkBaseConfig.encoder.inverted(inverted);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity) {
    if (m_trapezoidProfile.isPresent()) {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(maxVelocity.in(MetersPerSecond), m_config.getTrapezoidProfile().orElseThrow().maxAcceleration)));
    }
    m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(m_config.convertToMechanism(maxVelocity).in(RPM));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxVelocity(maxVelocity);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   */
  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration) {
    if (m_trapezoidProfile.isPresent()) {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxAcceleration.in(MetersPerSecondPerSecond))));
    }
    m_sparkBaseConfig.closedLoop.maxMotion.maxAcceleration(m_config.convertToMechanism(maxAcceleration).in(RPM.per(Second)));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxAcceleration(maxAcceleration);
      }
    });
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity) {
    if (m_trapezoidProfile.isPresent()) {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(maxVelocity.in(RotationsPerSecond), m_config.getTrapezoidProfile().orElseThrow().maxAcceleration)));
    }
    m_sparkBaseConfig.closedLoop.maxMotion.cruiseVelocity(maxVelocity.in(RPM));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxVelocity(maxVelocity);
      }
    });
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration) {
    if (m_trapezoidProfile.isPresent()) {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxAcceleration.in(RotationsPerSecondPerSecond))));
    }
    m_sparkBaseConfig.closedLoop.maxMotion.maxAcceleration(maxAcceleration.in(RPM.per(Second)));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxAcceleration(maxAcceleration);
      }
    });
  }

  @Override
  public void setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk) {
    // Only set when the trapezoid profile is velocity based.
    // Making
    // maxVelocity == maxAcceleration
    // maxAcceleration == maxJerk
    // TODO: Find a way to throw a wanring on this if trapezoidal profile isnt velocity based.
    if (m_trapezoidProfile.isPresent()) {
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxJerk.in(RotationsPerSecondPerSecond.per(Second)))));
    }
    m_sparkBaseConfig.closedLoop.maxMotion.maxAcceleration(maxJerk.in(RPM.per(Second).per(Second)));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxJerk(maxJerk);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if an exponential profile is configured with
   *                                                    a linear closed loop controller and the
   *                                                    mechanism circumference is not configured.
   */
  @Override
  public void setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput) {
    if (m_expoProfile.isPresent() && m_config.getExponentialProfile().isPresent()) {
      var exp = m_config.getExponentialProfile().get();
      var defaultkV = m_config.getLinearClosedLoopControllerUse() ? m_config.convertToMechanism(Meters.of(-exp.A / exp.B)).in(Rotations) : (-exp.A / exp.B);
      var defaultkA = m_config.getLinearClosedLoopControllerUse() ? m_config.convertToMechanism(Meters.of(1.0 / exp.B)).in(Rotations) : (1.0 / exp.B);
      var defaultMaxInput = exp.maxInput;
      m_expoProfile = Optional.of(new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(kV.orElse(defaultkV), kA.orElse(defaultkA), maxInput.orElse(Volts.of(defaultMaxInput)).in(Volts))));
      m_looseFollowers.ifPresent(smcs -> {
        for (var f : smcs) {
          f.setExponentialProfile(kV, kA, maxInput);
        }
      });
    }
  }

  @Override
  public void setKp(double kP) {
    m_config.getPID(m_slot).ifPresent(pid -> pid.setP(kP));
    m_pid.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
    });
    m_sparkBaseConfig.closedLoop.p(kP, m_closedLoopSlot);

    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKp(kP);
      }
    });
  }

  @Override
  public void setKi(double kI) {
    m_config.getPID(m_slot).ifPresent(simplePidController -> {
      simplePidController.setI(kI);
    });
    m_pid.ifPresent(simplePidController -> {
      simplePidController.setI(kI);
    });
    m_sparkBaseConfig.closedLoop.i(kI, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKi(kI);
      }
    });
  }

  @Override
  public void setKd(double kD) {
    m_config.getPID(m_slot).ifPresent(simplePidController -> {
      simplePidController.setD(kD);
    });
    m_pid.ifPresent(simplePidController -> {
      simplePidController.setD(kD);
    });
    m_sparkBaseConfig.closedLoop.d(kD, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKd(kD);
      }
    });
  }

  @Override
  public void setFeedback(double kP, double kI, double kD) {
    m_config.getPID(m_slot).ifPresent(simplePidController -> {
      simplePidController.setP(kP);
      simplePidController.setI(kI);
      simplePidController.setD(kD);
    });
    m_pid.ifPresent(simplePidController -> {
      simplePidController.setP(kP);
      simplePidController.setI(kI);
      simplePidController.setD(kD);
    });
    m_sparkBaseConfig.closedLoop.pid(kP, kI, kD, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setFeedback(kP, kI, kD);
      }
    });
  }

  @Override
  public void setKs(double kS) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKs(kS);
    });
    m_config.getArmFeedforward(m_slot).ifPresent(armFeedforward -> {
      armFeedforward.setKs(kS);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kS(kS, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKs(kS);
      }
    });
  }

  @Override
  public void setKv(double kV) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKv(kV);
    });
    m_config.getArmFeedforward(m_slot).ifPresent(armFeedforward -> {
      armFeedforward.setKv(kV);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKv(kV);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kV(kV, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKv(kV);
      }
    });
  }

  @Override
  public void setKa(double kA) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKa(kA);
    });
    m_config.getArmFeedforward(m_slot).ifPresent(armFeedforward -> {
      armFeedforward.setKa(kA);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKa(kA);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kA(kA, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKa(kA);
      }
    });
  }

  @Override
  public void setKg(double kG) {
    m_config.getArmFeedforward(m_slot).ifPresent(armFeedforward -> {
      armFeedforward.setKg(kG);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKg(kG);
    });
    if (m_config.getArmFeedforward(m_slot).isEmpty()) {
      m_sparkBaseConfig.closedLoop.feedForward.kG(kG, m_closedLoopSlot);
    } else {
      m_sparkBaseConfig.closedLoop.feedForward.kCos(kG, m_closedLoopSlot);
    }
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKg(kG);
      }
    });
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(simpleMotorFeedforward -> {
      simpleMotorFeedforward.setKs(kS);
      simpleMotorFeedforward.setKv(kV);
      simpleMotorFeedforward.setKa(kA);
    });
    m_config.getArmFeedforward(m_slot).ifPresent(armFeedforward -> {
      armFeedforward.setKs(kS);
      armFeedforward.setKv(kV);
      armFeedforward.setKa(kA);
      armFeedforward.setKg(kG);
      m_sparkBaseConfig.closedLoop.feedForward.kCos(kG, m_closedLoopSlot);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(elevatorFeedforward -> {
      elevatorFeedforward.setKs(kS);
      elevatorFeedforward.setKv(kV);
      elevatorFeedforward.setKa(kA);
      elevatorFeedforward.setKg(kG);
      m_sparkBaseConfig.closedLoop.feedForward.kG(kG, m_closedLoopSlot);
    });
    m_sparkBaseConfig.closedLoop.feedForward.kS(kS, m_closedLoopSlot).kV(kV, m_closedLoopSlot).kA(kA, m_closedLoopSlot);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setFeedforward(kS, kV, kA, kG);
      }
    });
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    m_config.withStatorCurrentLimit(currentLimit);
    m_sparkBaseConfig.smartCurrentLimit((int) currentLimit.in(Amps));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setStatorCurrentLimit(currentLimit);
      }
    });
  }

  @Deprecated
  /// Unsupported.
  public void setSupplyCurrentLimit(Current currentLimit) {
    //    m_looseFollowers.ifPresent(smcs -> {for(var f :
    //    smcs){f.setSupplyCurrentLimit(currentLimit);}});
  }

  @Override
  public void setClosedLoopRampRate(Time rampRate) {
    m_config.withClosedLoopRampRate(rampRate);
    m_sparkBaseConfig.closedLoopRampRate(rampRate.in(Seconds));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setClosedLoopRampRate(rampRate);
      }
    });
  }

  @Override
  public void setOpenLoopRampRate(Time rampRate) {
    m_config.withOpenLoopRampRate(rampRate);
    m_sparkBaseConfig.openLoopRampRate(rampRate.in(Seconds));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setOpenLoopRampRate(rampRate);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if a lower limit is configured and it is
   *                                                    greater than or equal to the new upper
   *                                                    limit.
   */
  @Override
  public void setMeasurementUpperLimit(Distance upperLimit) {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismLowerLimit().isPresent()) {
      m_config.withSoftLimits(m_config.convertFromMechanism(m_config.getMechanismLowerLimit().get()), upperLimit);
      m_sparkBaseConfig.softLimit.forwardSoftLimit(m_config.convertToMechanism(upperLimit).in(Rotations));
      m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
      m_looseFollowers.ifPresent(smcs -> {
        for (var f : smcs) {
          f.setMeasurementUpperLimit(upperLimit);
        }
      });
    }
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if an upper limit is configured and the new
   *                                                    lower limit is greater than or equal to it.
   */
  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit) {
    if (m_config.getMechanismCircumference().isPresent() && m_config.getMechanismUpperLimit().isPresent()) {
      m_config.withSoftLimits(lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
      m_sparkBaseConfig.softLimit.reverseSoftLimit(m_config.convertToMechanism(lowerLimit).in(Rotations));
      m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
      m_looseFollowers.ifPresent(smcs -> {
        for (var f : smcs) {
          f.setMeasurementLowerLimit(lowerLimit);
        }
      });
    }
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if a lower limit is configured and it is
   *                                                    greater than or equal to the new upper
   *                                                    limit.
   */
  @Override
  public void setMechanismUpperLimit(Angle upperLimit) {
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> {
      m_config.withSoftLimits(lowerLimit, upperLimit);
    });
    m_sparkBaseConfig.softLimit.forwardSoftLimit(upperLimit.in(Rotations));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismUpperLimit(upperLimit);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if an upper limit is configured and the new
   *                                                    lower limit is greater than or equal to it.
   */
  @Override
  public void setMechanismLowerLimit(Angle lowerLimit) {
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> {
      m_config.withSoftLimits(lowerLimit, upperLimit);
    });
    m_sparkBaseConfig.softLimit.reverseSoftLimit(lowerLimit.in(Rotations));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismLowerLimit(lowerLimit);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if both limits are non-null and the lower
   *                                                    limit is greater than or equal to the upper
   *                                                    limit.
   */
  @Override
  public void setMechanismLimits(Angle lower, Angle upper) {
    m_config.withSoftLimits(lower, upper);
    m_sparkBaseConfig.softLimit.reverseSoftLimit(lower.in(Rotations)).forwardSoftLimit(upper.in(Rotations));
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismLimits(lower, upper);
      }
    });
  }

  @Override
  public void setMechanismLimitsEnabled(boolean enabled) {
    m_sparkBaseConfig.softLimit.forwardSoftLimitEnabled(enabled).reverseSoftLimitEnabled(enabled);
    m_spark.configureAsync(m_sparkBaseConfig, ResetMode.kNoResetSafeParameters, DriverStationBackend.isEnabled() ? PersistMode.kNoPersistParameters : PersistMode.kPersistParameters);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismLimitsEnabled(enabled);
      }
    });
  }

  @Override
  public void setMechanismGearing(MechanismGearing gearing) {
    m_config.withGearing(gearing);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismGearing(gearing);
      }
    });
  }

  @Override
  public void setMechanismCircumference(Distance circumference) {
    m_config.withMechanismCircumference(circumference);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismCircumference(circumference);
      }
    });
  }

  /**
   * Convert generic slot into spark specific slot.
   *
   * @param slot {@link ClosedLoopControllerSlot} to convert
   * @return spark specific slot {@link ClosedLoopSlot}
   * @throws IllegalArgumentException if the slot is not one of SLOT_0 through SLOT_3.
   */
  private ClosedLoopSlot getSparkClosedLoopSlot(ClosedLoopControllerSlot slot) {
    switch (slot) {
      case SLOT_0:
        return ClosedLoopSlot.kSlot0;
      case SLOT_1:
        return ClosedLoopSlot.kSlot1;
      case SLOT_2:
        return ClosedLoopSlot.kSlot2;
      case SLOT_3:
        return ClosedLoopSlot.kSlot3;
      default:
        throw new IllegalArgumentException("Invalid slot: " + slot);
    }
  }

  @Override
  public void setClosedLoopSlot(ClosedLoopControllerSlot slot) {
    m_slot = slot;
    m_closedLoopSlot = getSparkClosedLoopSlot(slot);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setClosedLoopSlot(slot);
      }
    });
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.of(m_spark.getMotorTemperature().get());
  }

  @Override
  public SmartMotorControllerConfig<?> getConfig() {
    return m_config;
  }

  @Override
  public Object getMotorController() {
    return m_spark;
  }

  @Override
  public Object getMotorControllerConfig() {
    return m_sparkBaseConfig;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields() {
    return Pair.of(Optional.empty(), Optional.of(List.of(DoubleTelemetryField.SupplyCurrent, DoubleTelemetryField.SupplyCurrentLimit)));
  }

  @Override
  public void close() {
    super.close();
    m_externalEncoderGearingDiscontinuityAlert.close();
  }
}
