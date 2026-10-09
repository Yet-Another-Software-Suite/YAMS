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
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.thrifty.canEncoder.CanEncoder;
import com.thrifty.canEncoder.CanEncoderConfig;
import com.thrifty.core.Motor.Direction;
import com.thrifty.core.Motor.FeedbackSensorType;
import com.thrifty.nova.Nova;
import com.thrifty.nova.NovaConfig;
import com.thrifty.nova.NovaConfigBatch;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.math.trajectory.ExponentialProfile;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.trajectory.TrapezoidProfile.Constraints;
import org.wpilib.math.util.MathUtil;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.system.Notifier;
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
 * Nova wrapper for The Thrifty Bot's Thrifty Nova motor controller, using ThriftyLib 2027.
 *
 * <p>The closed loop controller runs on the SystemCore and commands the Nova with voltage, like the
 * profiled controllers of the other wrappers. ThriftyLib's simulation ignores the Nova's onboard
 * PID gains, so running the loop on the SystemCore keeps simulation and the robot the same.
 *
 * <p><b>External encoders</b>, given to
 * {@link SmartMotorControllerConfig#withExternalEncoder(Object)}:
 *
 * <ul>
 * <li>{@link FeedbackSensorType#ABS}: an absolute encoder wired to the Nova's data port. Its
 * type comes from the vendor config, such as
 * {@code NovaConfig.absoluteEncoderType(Motor.AbsoluteEncoderType.REV_ENCODER)}.
 * <li>{@link FeedbackSensorType#QUAD}: a quadrature encoder wired to the Nova's data port. Its
 * type comes from the vendor config, such as
 * {@code NovaConfig.quadratureEncoderType(Motor.QuadratureEncoderType.custom(2048))}.
 * <li>{@link CanEncoder}: a Thrifty CAN Encoder on the same CAN bus.
 * </ul>
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // Configure and create a Nova driving a NEO on CAN bus 0, CAN ID 3
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withMotorInverted(false)
 *     .withStatorCurrentLimit(Amps.of(40))
 *     .withClosedLoopController(0.2, 0, 0)
 *     .withVendorConfig(new NovaConfigBatch(NovaConfig.tempThrottleEnable(true)));
 * SmartMotorController motor = new NovaWrapper(new Nova(0, 3, MotorType.NEO),
 *     DCMotor.getNEO(1), config);
 * }</pre>
 */
public class NovaWrapper extends SmartMotorController {
  /** Thrifty Nova controller. */
  private final Nova m_nova;
  /** Vendor config applied to the Nova before the YAMS config. */
  private final Optional<NovaConfig.Config> m_novaVendorConfig;
  /** Motor characteristics controlled by the {@link Nova}. */
  private final DCMotor m_motor;
  /** Sim for the Nova. */
  private Optional<DCMotorSim> m_dcMotorSim = Optional.empty();
  /** Data port encoder used as the external encoder, {@link FeedbackSensorType#ABS} or {@link FeedbackSensorType#QUAD}. */
  private Optional<FeedbackSensorType> m_dataPortEncoder = Optional.empty();
  /** CAN encoder used as the external encoder. */
  private Optional<CanEncoder> m_canEncoder = Optional.empty();
  /**
   * Discontinuity point of the external absolute encoder: it reports angles from one rotation below
   * it up to it, so [-0.5, 0.5) rotations for 0.5 rotations, or [0, 1) for the default of 1 rotation.
   */
  private Angle m_absoluteEncoderDiscontinuityPoint = Rotations.of(1);
  /** Acceleration filter. */
  private final DerivativeTimeFilter m_accelerationFilter = new DerivativeTimeFilter(Milliseconds.of(20));
  /**
   * Duty cycle last applied to the simulated motor. The simulation holds it between commands, as the
   * Nova holds its output, so it must be what was commanded and not read back from the motor model.
   */
  private volatile double m_simDutyCycle = 0;

  /**
   * Construct the Nova Wrapper for the generic {@link SmartMotorController}.
   *
   * @param controller {@link Nova} to use.
   * @param motor      {@link DCMotor} connected to the {@link Nova}.
   * @param config     {@link SmartMotorControllerConfig} to apply to the {@link Nova}. Its vendor
   *                   config, if any, must be a {@link NovaConfig.Config}, such as a
   *                   {@link com.thrifty.nova.NovaConfigBatch}, and is applied before the YAMS
   *                   config.
   * @throws SmartMotorControllerConfigurationException if the vendor config is not a
   *                                                    {@link NovaConfig.Config}.
   * @throws SmartMotorControllerConfigurationException if
   *                                                    {@link #applyConfig(SmartMotorControllerConfig)}
   *                                                    rejects the config.
   * @throws IllegalArgumentException if {@link #applyConfig(SmartMotorControllerConfig)} rejects
   *                                  the config.
   */
  public NovaWrapper(Nova controller, DCMotor motor, SmartMotorControllerConfig<?> config) {
    if (config.getVendorConfig().isPresent()) {
      var genCfg = config.getVendorConfig().get();
      if (!(genCfg instanceof NovaConfig.Config novaConfig)) {
        throw new SmartMotorControllerConfigurationException("NovaConfig.Config is the only acceptable vendor config for Nova controllers.", "NovaConfig.Config not found.", ".withVendorConfig(new NovaConfigBatch(...))");
      }
      m_novaVendorConfig = Optional.of(novaConfig);
    } else {
      m_novaVendorConfig = Optional.empty();
    }
    m_nova = controller;
    m_motor = motor;
    m_config = config;
    m_systemCoreClosedLoopAlert = Optional.of(new Alert("YAMS", buildAlertId("Nova", m_nova.status().getCanId(), "ClosedLoop"), getName() + " closed loop controller is running on the SystemCore.", Alert.Level.MEDIUM));
    setupSimulation();
    try {
      applyConfig(config);
      checkConfigSafety();
    } catch (RuntimeException e) {
      // Release what applying the config started, such as the closed loop controller's Notifier,
      // which would otherwise keep running for a motor controller that was never made.
      close();
      throw e;
    }
  }

  /**
   * Direction a Thrifty device counts as positive.
   *
   * @param inverted Whether the device is inverted from its default, clockwise, direction.
   * @return {@link Direction} of the device.
   */
  private static Direction getDirection(boolean inverted) {
    return inverted ? Direction.COUNTER_CLOCKWISE : Direction.CLOCKWISE;
  }

  @Override
  public void setupSimulation() {
    if (RobotBase.isSimulation()) {
      if (m_dcMotorSim.isEmpty()) {
        m_dcMotorSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(m_motor, m_config.getMOI(), m_config.getGearing().getMechanismToRotorRatio()), m_motor));
        setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
      }
      m_config.getStartingPosition().ifPresent(startingPos -> m_simSupplier.ifPresent(sim -> sim.setMechanismPosition(startingPos)));
    }
  }

  /**
   * Get the angle of the external encoder's shaft.
   *
   * @return Angle of the external encoder's shaft, or empty without an external encoder.
   */
  private Optional<Angle> getExternalEncoderSensorPosition() {
    if (m_dataPortEncoder.isEmpty() && m_canEncoder.isEmpty()) {
      return Optional.empty();
    }
    final boolean absolute = m_canEncoder.isPresent() || m_dataPortEncoder.get() == FeedbackSensorType.ABS;
    final double rotations;
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      // ThriftyLib's simulation does not move the external encoders with the mechanism.
      rotations = m_simSupplier.get().getMechanismPosition().times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()).in(Rotations);
    } else if (m_canEncoder.isPresent()) {
      rotations = m_canEncoder.get().status().getPositionAbs();
    } else if (absolute) {
      rotations = m_nova.status().getAbsPosition();
    } else {
      rotations = m_nova.status().getQuadPosition();
    }
    // An absolute encoder reports angles within one rotation, below its discontinuity point.
    return Optional.of(Rotations.of(absolute
                                    ? MathUtil.inputModulus(rotations, m_absoluteEncoderDiscontinuityPoint.in(Rotations) - 1, m_absoluteEncoderDiscontinuityPoint.in(Rotations))
                                    : rotations));
  }

  /**
   * Get the angular velocity of the external encoder's shaft.
   *
   * @return Angular velocity of the external encoder's shaft, or empty without an external encoder.
   */
  private Optional<AngularVelocity> getExternalEncoderSensorVelocity() {
    if (m_dataPortEncoder.isEmpty() && m_canEncoder.isEmpty()) {
      return Optional.empty();
    }
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      return Optional.of(m_simSupplier.get().getMechanismVelocity().times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()));
    }
    if (m_canEncoder.isPresent()) {
      return Optional.of(RotationsPerSecond.of(m_canEncoder.get().status().getVelocity()));
    }
    // The absolute encoder's velocity reads 0 until NovaConfig.absoluteFramePeriod enables its frame.
    return Optional.of(RotationsPerSecond.of(m_dataPortEncoder.get() == FeedbackSensorType.ABS ? m_nova.status().getAbsVelocity() : m_nova.status().getQuadVelocity()));
  }

  @Override
  public void seedRelativeEncoder() {
    getExternalEncoderMechanismPosition().ifPresent(mechanismAngle -> {
      final double rotorRotations = mechanismAngle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations);
      m_nova.setIntPosition(rotorRotations);
    });
  }

  @Override
  public void synchronizeRelativeEncoder() {
    if (m_config.getFeedbackSynchronizationThreshold().isPresent()) {
      final Optional<Angle> externalEncoderAngle = getExternalEncoderMechanismPosition();
      if (externalEncoderAngle.isPresent() && !getRelativeMechanismPosition().isNear(externalEncoderAngle.get(), m_config.getFeedbackSynchronizationThreshold().get())) {
        seedRelativeEncoder();
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
      m_nova.simulationPeriodic();
      m_canEncoder.ifPresent(CanEncoder::simulationPeriodic);
      // TODO: Uncomment after the 2026 season
      //      m_looseFollowers.ifPresent(smcs -> {for(var f : smcs){f.simIterate();}});
    }
  }

  @Override
  public void setZeroPower(MotorMode mode) {
    m_nova.configure(NovaConfig.brakeMode(mode == MotorMode.BRAKE));
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException if called outside of simulation, since Thrifty Novas do
   *                                       not support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(AngularVelocity velocity) {
    if (!RobotBase.isSimulation()) {
      throw new UnsupportedOperationException("Thrifty Nova does not support setting encoder velocity.");
    }
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismVelocity(velocity));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *                                                    configured.
   * @throws UnsupportedOperationException if called outside of simulation, since Thrifty Novas do
   *                                       not support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(LinearVelocity velocity) {
    setEncoderVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    final double externalEncoderRotations = angle.times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()).in(Rotations);
    m_dataPortEncoder.ifPresent(encoder -> {
      if (encoder == FeedbackSensorType.ABS) {
        m_nova.setAbsPosition(externalEncoderRotations);
      } else {
        m_nova.setQuadPosition(externalEncoderRotations);
      }
    });
    if (!RobotBase.isSimulation()) {
      m_canEncoder.ifPresent(canEncoder -> {
        // Move the zero offset so the encoder reads the angle where it is now.
        final var status = canEncoder.status();
        canEncoder.configure(CanEncoderConfig.zeroOffset(MathUtil.inputModulus(status.getPositionAbs() + status.getZeroOffset() - externalEncoderRotations, 0, 1)));
      });
    }
    m_nova.setIntPosition(angle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
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
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setVelocity(angularVelocity);
      }
    });
  }

  @Override
  public void setVelocity(AngularVelocity angularVelocity, Force feedforwardForce) {
    setpointPosition = Optional.empty();
    setpointVelocity = Optional.ofNullable(angularVelocity);
    setpointFeedforwardForce = Optional.ofNullable(feedforwardForce);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setVelocity(angularVelocity, feedforwardForce);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if different open and closed loop ramp
   *                                                    rates are configured.
   * @throws SmartMotorControllerConfigurationException if a zero offset or discontinuity point is
   *                                                    configured for a quadrature encoder.
   * @throws SmartMotorControllerConfigurationException if an external encoder option is configured
   *                                                    without an external encoder.
   * @throws SmartMotorControllerConfigurationException if a vendor control request is configured.
   * @throws SmartMotorControllerConfigurationException if a required config option is left
   *                                                    unhandled during validation.
   * @throws IllegalArgumentException if a closed loop control period is configured while not in
   *                                  closed loop mode.
   * @throws IllegalArgumentException if a closed loop tolerance is configured with an LQR
   *                                  controller.
   * @throws IllegalArgumentException if the external encoder is not
   *                                  {@link FeedbackSensorType#ABS},
   *                                  {@link FeedbackSensorType#QUAD} or a {@link CanEncoder}.
   * @throws IllegalArgumentException if a follower is not a {@link Nova}.
   * @throws IllegalArgumentException if relative encoder inversion is configured.
   */
  @Override
  public boolean applyConfig(SmartMotorControllerConfig<?> config) {
    m_config = config;
    config.resetValidationCheck();
    m_systemCoreClosedLoopAlert.ifPresent(alert -> alert.set(false));
    m_lqr = config.getLQRClosedLoopController();
    m_pid = config.getPID(m_slot);
    m_looseFollowers = config.getLooselyCoupledFollowers();

    // Settings are sent to the Nova together, in order: reset the Nova if configured, then the
    // vendor config, then the YAMS config, which overrides the vendor config.
    final NovaConfigBatch novaConfig = new NovaConfigBatch();
    if (config.getResetPreviousConfig()) {
      novaConfig.add(NovaConfig.factoryReset());
    }
    m_novaVendorConfig.ifPresent(novaConfig::add);

    // Handle motion profiles; they and the feedforwards run in the closed loop controller.
    m_expoProfile = config.getExponentialProfile().map(ExponentialProfile::new);
    m_trapezoidProfile = config.getTrapezoidProfile().map(TrapezoidProfile::new);
    for (var closedLoopControlSlot : ClosedLoopControllerSlot.values()) {
      config.getPID(closedLoopControlSlot);
      config.getArmFeedforward(closedLoopControlSlot);
      config.getElevatorFeedforward(closedLoopControlSlot);
      config.getSimpleFeedforward(closedLoopControlSlot);
    }
    config.getGearing();
    config.getMechanismLowerLimit();
    config.getMechanismUpperLimit();
    config.getTemperatureCutoff();
    config.getClosedLoopControllerMaximumVoltage();
    config.getFeedbackSynchronizationThreshold();

    // LQR doesn't handle tolerances
    if (m_lqr.isPresent() && config.getClosedLoopTolerance().isPresent()) {
      throw new IllegalArgumentException("[Error] Closed loop tolerance is not supported in LQR mode.");
    }
    config.getClosedLoopTolerance().ifPresent(tolerance -> {
      if (config.getLinearClosedLoopControllerUse()) {
        m_pid.ifPresent(pidController -> pidController.setTolerance(config.convertFromMechanism(tolerance).in(Meters)));
      } else {
        m_pid.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
      }
    });

    // The closed loop controller runs on the SystemCore.
    if (m_closedLoopControllerThread == null) {
      m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
    } else {
      stopClosedLoopController();
      m_closedLoopControllerThread.close();
      m_closedLoopControllerThread = new Notifier(this::iterateClosedLoopController);
    }
    if (config.getTelemetryName().isPresent()) {
      m_closedLoopControllerThread.setName(config.getTelemetryName().get());
    }

    // Ramp rates. The Nova has one ramp rate, applied to the SystemCore's closed loop output too.
    Optional<Time> rampRate = config.getOpenLoopRampRate();
    Optional<Time> closedLoopRampRate = config.getClosedLoopRampRate();
    if (rampRate.isPresent() && closedLoopRampRate.isPresent() && !rampRate.get().isEquivalent(closedLoopRampRate.get())) {
      throw new SmartMotorControllerConfigurationException("Thrifty Nova has one ramp rate for open and closed loop control", "Different open and closed loop ramp rates could not be applied", ".withOpenLoopRampRate or .withClosedLoopRampRate, not both");
    }
    rampRate.or(() -> closedLoopRampRate).ifPresent(rate -> novaConfig.add(NovaConfig.rampForward(rate.in(Seconds)), NovaConfig.rampReverse(rate.in(Seconds))));

    // Inversions
    config.getMotorInverted().ifPresent(inverted -> novaConfig.add(NovaConfig.direction(getDirection(inverted))));
    if (config.getEncoderInverted().isPresent()) {
      throw new IllegalArgumentException("[ERROR] Thrifty Nova internal encoder cannot be inverted!");
    }

    // Current limits
    config.getSupplyStallCurrentLimit().ifPresent(limit -> novaConfig.add(NovaConfig.supplyCurrent(limit)));
    config.getStatorStallCurrentLimit().ifPresent(limit -> novaConfig.add(NovaConfig.statorCurrent(limit)));

    // Voltage Compensation
    config.getVoltageCompensation().ifPresent(voltage -> novaConfig.add(NovaConfig.voltageComp(voltage.in(Volts))));

    // Zero power mode
    config.getZeroPower().ifPresent(mode -> novaConfig.add(NovaConfig.brakeMode(mode == MotorMode.BRAKE)));

    // Continuous wrapping is handled by the closed loop controller.
    config.getContinuousWrapping();
    config.getContinuousWrappingMin();

    // External encoder
    m_dataPortEncoder = Optional.empty();
    m_canEncoder = Optional.empty();
    m_absoluteEncoderDiscontinuityPoint = Rotations.of(1);
    if (config.getExternalEncoder().isPresent()) {
      final double mechToEncoder = config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio();
      final Object externalEncoder = config.getExternalEncoder().get();
      m_absoluteEncoderDiscontinuityPoint = config.getExternalEncoderDiscontinuityPoint().orElse(Rotations.of(1));
      config.getUseExternalFeedback();
      if (externalEncoder == FeedbackSensorType.ABS) {
        m_dataPortEncoder = Optional.of(FeedbackSensorType.ABS);
        config.getExternalEncoderInverted().ifPresent(inverted -> novaConfig.add(NovaConfig.absoluteDirection(getDirection(inverted))));
        config.getExternalEncoderZeroOffset().ifPresent(offset -> novaConfig.add(NovaConfig.absOffset(MathUtil.inputModulus(offset.times(mechToEncoder).in(Rotations), 0, 1))));
      } else if (externalEncoder == FeedbackSensorType.QUAD) {
        m_dataPortEncoder = Optional.of(FeedbackSensorType.QUAD);
        if (config.getExternalEncoderZeroOffset().isPresent()) {
          throw new SmartMotorControllerConfigurationException("Zero offset is only available for absolute encoders", "Zero offset could not be applied to the quadrature encoder", ".withExternalEncoderZeroOffset");
        }
        if (config.getExternalEncoderDiscontinuityPoint().isPresent()) {
          throw new SmartMotorControllerConfigurationException("Discontinuity point is only available for absolute encoders", "Discontinuity point could not be applied to the quadrature encoder", ".withExternalEncoderDiscontinuityPoint");
        }
        config.getExternalEncoderInverted().ifPresent(inverted -> novaConfig.add(NovaConfig.quadratureDirection(getDirection(inverted))));
      } else if (externalEncoder instanceof CanEncoder canEncoder) {
        m_canEncoder = Optional.of(canEncoder);
        config.getExternalEncoderInverted().ifPresent(inverted -> canEncoder.configure(CanEncoderConfig.direction(getDirection(inverted))));
        config.getExternalEncoderZeroOffset().ifPresent(offset -> canEncoder.configure(CanEncoderConfig.zeroOffset(MathUtil.inputModulus(offset.times(mechToEncoder).in(Rotations), 0, 1))));
      } else {
        throw new IllegalArgumentException("[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName() + ".\n\tPlease use FeedbackSensorType.ABS, FeedbackSensorType.QUAD or a CanEncoder instead.");
      }
    } else {
      if (config.getExternalEncoderDiscontinuityPoint().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder discontinuity point is only available for external encoders", "External encoder discontinuity point could not be applied", ".withExternalEncoderDiscontinuityPoint");
      }
      if (config.getExternalEncoderZeroOffset().isPresent()) {
        throw new SmartMotorControllerConfigurationException("Zero offset is only available for external encoders", "Zero offset could not be applied", ".withExternalEncoderZeroOffset");
      }
      if (config.getExternalEncoderInverted().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder cannot be inverted because no external encoder exists", "External encoder could not be inverted", ".withExternalEncoderInverted");
      }
      if (config.getExternalEncoderGearing().isPresent()) {
        throw new SmartMotorControllerConfigurationException("External encoder gearing is not supported when there is no external encoder", "External encoder gearing could not be set", ".withExternalEncoderGearing");
      }
      config.getUseExternalFeedback();
    }

    m_nova.configure(novaConfig);

    // A quadrature encoder counts from where it powers on, like the motor's encoder.
    if (m_dataPortEncoder.filter(encoder -> encoder == FeedbackSensorType.QUAD).isPresent()) {
      m_nova.setQuadPosition(config.getStartingPosition().orElse(Rotations.zero()).times(config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getMechanismToRotorRatio()).in(Rotations));
    }

    // Starting position, or the absolute encoder's position without one.
    if (config.getStartingPosition().isPresent()) {
      m_nova.setIntPosition(config.getStartingPosition().get().times(config.getGearing().getMechanismToRotorRatio()).in(Rotations));
      m_simSupplier.ifPresent(sim -> sim.setMechanismPosition(config.getStartingPosition().get()));
    } else if (m_canEncoder.isPresent() || m_dataPortEncoder.filter(encoder -> encoder == FeedbackSensorType.ABS).isPresent()) {
      seedRelativeEncoder();
    }

    // Followers
    if (config.getFollowers().isPresent()) {
      for (Pair<Object, Boolean> follower : config.getFollowers().get()) {
        if (follower.getFirst() instanceof Nova followerNova) {
          followerNova.configure(NovaConfig.follow(m_nova.status().getCanId()));
          followerNova.setInverted(follower.getSecond());
          config.getZeroPower().ifPresent(mode -> followerNova.configure(NovaConfig.brakeMode(mode == MotorMode.BRAKE)));
        } else {
          throw new IllegalArgumentException("[ERROR] Unknown follower type: " + follower.getFirst().getClass().getSimpleName());
        }
      }
      config.clearFollowers();
    }

    if (config.getVendorControlRequest().isPresent()) {
      throw new SmartMotorControllerConfigurationException("Nova(" + m_nova.status().getCanId() + ") does not support the custom control requests!", "Cannot use given control request", "withVendorControlRequest()");
    }

    if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP) {
      m_systemCoreClosedLoopAlert.ifPresent(alert -> alert.set(true));
      startClosedLoopController();
    } else if (config.getClosedLoopControlPeriod().isPresent()) {
      throw new IllegalArgumentException("[Error] Closed loop control period is only supported in closed loop mode.");
    }

    config.validateBasicOptions();
    config.validateExternalEncoderOptions();
    return true;
  }

  @Override
  public double getDutyCycle() {
    return m_simSupplier.isPresent() ? m_simDutyCycle : m_nova.status().getAppliedPower();
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    m_nova.setThrottle(dutyCycle);
    m_simSupplier.ifPresent(simSupplier -> {
      m_simDutyCycle = Math.clamp(dutyCycle, -1, 1);
      simSupplier.setMechanismStatorDutyCycle(m_simDutyCycle);
    });
    if (dutyCycle == 0.0) {
      m_looseFollowers.ifPresent(looseFollower -> {
        for (var follower : looseFollower) {
          follower.setDutyCycle(dutyCycle);
        }
      });
    }
  }

  @Override
  public Optional<Current> getSupplyCurrent() {
    return Optional.of(m_simSupplier.map(simSupplier -> simSupplier.getSupplyCurrent()).orElseGet(() -> Amps.of(m_nova.status().getCurrentSupply())));
  }

  @Override
  public Current getStatorCurrent() {
    return m_simSupplier.map(simSupplier -> simSupplier.getStatorCurrent()).orElseGet(() -> Amps.of(m_nova.status().getCurrentStator()));
  }

  @Override
  public Voltage getVoltage() {
    return m_simSupplier.map(simSupplier -> simSupplier.getMechanismSupplyVoltage().times(m_simDutyCycle)).orElseGet(() -> Volts.of(m_nova.status().getAppliedVoltage()));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_nova.setVoltage(voltage);
    m_simSupplier.ifPresent(simSupplier -> {
      // The Nova cannot apply more than its supply voltage.
      final Voltage supplyVoltage = simSupplier.getMechanismSupplyVoltage();
      m_simDutyCycle = Math.clamp(voltage.in(Volts) / supplyVoltage.in(Volts), -1, 1);
      simSupplier.setMechanismStatorVoltage(supplyVoltage.times(m_simDutyCycle));
    });
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
    if (m_config.getUseExternalFeedback()) {
      final Optional<AngularVelocity> externalEncoderVelocity = getExternalEncoderMechanismVelocity();
      if (externalEncoderVelocity.isPresent()) {
        return externalEncoderVelocity.get();
      }
    }
    return getRelativeMechanismVelocity();
  }

  @Override
  public AngularAcceleration getMechanismAcceleration() {
    return RotationsPerSecond.per(Microsecond).of(m_accelerationFilter.derivative(getMechanismVelocity().in(RotationsPerSecond)));
  }

  @Override
  public Angle getMechanismPosition() {
    if (m_config.getUseExternalFeedback()) {
      final Optional<Angle> externalEncoderPosition = getExternalEncoderMechanismPosition();
      if (externalEncoderPosition.isPresent()) {
        return externalEncoderPosition.get();
      }
    }
    return getRelativeMechanismPosition();
  }

  @Override
  public AngularVelocity getRelativeMechanismVelocity() {
    return getRotorVelocity().times(m_config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public Angle getRelativeMechanismPosition() {
    return getRotorPosition().times(m_config.getGearing().getRotorToMechanismRatio());
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      return m_simSupplier.get().getRotorVelocity();
    }
    return RotationsPerSecond.of(m_nova.status().getIntVelocity());
  }

  @Override
  public Angle getRotorPosition() {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      return m_simSupplier.get().getRotorPosition();
    }
    return Rotations.of(m_nova.status().getIntPosition());
  }

  @Override
  public Optional<Angle> getExternalEncoderMechanismPosition() {
    return getExternalEncoderSensorPosition().map(angle -> angle.times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio()));
  }

  @Override
  public Optional<AngularVelocity> getExternalEncoderMechanismVelocity() {
    return getExternalEncoderSensorVelocity().map(velocity -> velocity.times(m_config.getExternalEncoderGearing().orElse(MechanismGearing.kOne).getRotorToMechanismRatio()));
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    m_config.withMotorInverted(inverted);
    m_nova.setInverted(inverted);
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the Thrifty Nova's internal encoder
   *                                       cannot be inverted.
   */
  @Override
  public void setEncoderInverted(boolean inverted) {
    throw new UnsupportedOperationException("Thrifty Nova internal encoder cannot be inverted.");
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
      final Constraints constraints = new Constraints(maxVelocity.in(MetersPerSecond), m_config.getTrapezoidProfile().orElseThrow().maxAcceleration);
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withTrapezoidalProfileConstraints(constraints);
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(constraints));
    }
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
      final Constraints constraints = new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxAcceleration.in(MetersPerSecondPerSecond));
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withTrapezoidalProfileConstraints(constraints);
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(constraints));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxAcceleration(maxAcceleration);
      }
    });
  }

  @Override
  public void setMotionProfileMaxVelocity(AngularVelocity maxVelocity) {
    if (m_trapezoidProfile.isPresent()) {
      final Constraints constraints = new Constraints(maxVelocity.in(RotationsPerSecond), m_config.getTrapezoidProfile().orElseThrow().maxAcceleration);
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withTrapezoidalProfileConstraints(constraints);
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(constraints));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxVelocity(maxVelocity);
      }
    });
  }

  @Override
  public void setMotionProfileMaxAcceleration(AngularAcceleration maxAcceleration) {
    if (m_trapezoidProfile.isPresent()) {
      final Constraints constraints = new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxAcceleration.in(RotationsPerSecondPerSecond));
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withTrapezoidalProfileConstraints(constraints);
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(constraints));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxAcceleration(maxAcceleration);
      }
    });
  }

  @Override
  public void setMotionProfileMaxJerk(Velocity<AngularAccelerationUnit> maxJerk) {
    // Only meaningful when the trapezoid profile is velocity based, where its velocity is the
    // mechanism's acceleration and its acceleration the mechanism's jerk.
    if (m_trapezoidProfile.isPresent()) {
      final Constraints constraints = new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity, maxJerk.in(RotationsPerSecondPerSecond.per(Second)));
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withTrapezoidalProfileConstraints(constraints);
      m_trapezoidProfile = Optional.of(new TrapezoidProfile(constraints));
    }
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMotionProfileMaxJerk(maxJerk);
      }
    });
  }

  @Override
  public void setExponentialProfile(OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput) {
    if (m_expoProfile.isPresent() && m_config.getExponentialProfile().isPresent()) {
      var exp = m_config.getExponentialProfile().get();
      // kV and kA are in the profile's units, like its constraints.
      var defaultkV = -exp.A / exp.B;
      var defaultkA = 1.0 / exp.B;
      var defaultMaxInput = exp.maxInput;
      final ExponentialProfile.Constraints constraints = ExponentialProfile.Constraints.fromCharacteristics(maxInput.orElse(Volts.of(defaultMaxInput)).in(Volts), kV.orElse(defaultkV), kA.orElse(defaultkA));
      // Keep the tuned constraints in the config, so tuning the next one keeps this one.
      m_config.withExponentialProfile(constraints);
      m_expoProfile = Optional.of(new ExponentialProfile(constraints));
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
    m_pid.ifPresent(pid -> pid.setP(kP));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKp(kP);
      }
    });
  }

  @Override
  public void setKi(double kI) {
    m_config.getPID(m_slot).ifPresent(pid -> pid.setI(kI));
    m_pid.ifPresent(pid -> pid.setI(kI));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKi(kI);
      }
    });
  }

  @Override
  public void setKd(double kD) {
    m_config.getPID(m_slot).ifPresent(pid -> pid.setD(kD));
    m_pid.ifPresent(pid -> pid.setD(kD));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKd(kD);
      }
    });
  }

  @Override
  public void setFeedback(double kP, double kI, double kD) {
    m_config.getPID(m_slot).ifPresent(pid -> pid.setPID(kP, kI, kD));
    m_pid.ifPresent(pid -> pid.setPID(kP, kI, kD));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setFeedback(kP, kI, kD);
      }
    });
  }

  @Override
  public void setKs(double kS) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(ff -> ff.setKs(kS));
    m_config.getArmFeedforward(m_slot).ifPresent(ff -> ff.setKs(kS));
    m_config.getElevatorFeedforward(m_slot).ifPresent(ff -> ff.setKs(kS));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKs(kS);
      }
    });
  }

  @Override
  public void setKv(double kV) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(ff -> ff.setKv(kV));
    m_config.getArmFeedforward(m_slot).ifPresent(ff -> ff.setKv(kV));
    m_config.getElevatorFeedforward(m_slot).ifPresent(ff -> ff.setKv(kV));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKv(kV);
      }
    });
  }

  @Override
  public void setKa(double kA) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(ff -> ff.setKa(kA));
    m_config.getArmFeedforward(m_slot).ifPresent(ff -> ff.setKa(kA));
    m_config.getElevatorFeedforward(m_slot).ifPresent(ff -> ff.setKa(kA));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKa(kA);
      }
    });
  }

  @Override
  public void setKg(double kG) {
    m_config.getArmFeedforward(m_slot).ifPresent(ff -> ff.setKg(kG));
    m_config.getElevatorFeedforward(m_slot).ifPresent(ff -> ff.setKg(kG));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setKg(kG);
      }
    });
  }

  @Override
  public void setFeedforward(double kS, double kV, double kA, double kG) {
    m_config.getSimpleFeedforward(m_slot).ifPresent(ff -> {
      ff.setKs(kS);
      ff.setKv(kV);
      ff.setKa(kA);
    });
    m_config.getArmFeedforward(m_slot).ifPresent(ff -> {
      ff.setKs(kS);
      ff.setKv(kV);
      ff.setKa(kA);
      ff.setKg(kG);
    });
    m_config.getElevatorFeedforward(m_slot).ifPresent(ff -> {
      ff.setKs(kS);
      ff.setKv(kV);
      ff.setKa(kA);
      ff.setKg(kG);
    });
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setFeedforward(kS, kV, kA, kG);
      }
    });
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    m_config.withStatorCurrentLimit(currentLimit);
    m_nova.configure(NovaConfig.statorCurrent(currentLimit.in(Amps)));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setStatorCurrentLimit(currentLimit);
      }
    });
  }

  @Override
  public void setSupplyCurrentLimit(Current currentLimit) {
    m_config.withSupplyCurrentLimit(currentLimit);
    m_nova.configure(NovaConfig.supplyCurrent(currentLimit.in(Amps)));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setSupplyCurrentLimit(currentLimit);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * <p>The Thrifty Nova has one ramp rate, for both open and closed loop control.
   */
  @Override
  public void setClosedLoopRampRate(Time rampRate) {
    m_config.withClosedLoopRampRate(rampRate);
    m_nova.configure(NovaConfig.rampForward(rampRate.in(Seconds)), NovaConfig.rampReverse(rampRate.in(Seconds)));
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setClosedLoopRampRate(rampRate);
      }
    });
  }

  /**
   * {@inheritDoc}
   *
   * <p>The Thrifty Nova has one ramp rate, for both open and closed loop control.
   */
  @Override
  public void setOpenLoopRampRate(Time rampRate) {
    m_config.withOpenLoopRampRate(rampRate);
    m_nova.configure(NovaConfig.rampForward(rampRate.in(Seconds)), NovaConfig.rampReverse(rampRate.in(Seconds)));
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
    m_config.getMechanismLowerLimit().ifPresent(lowerLimit -> m_config.withSoftLimits(lowerLimit, upperLimit));
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
    m_config.getMechanismUpperLimit().ifPresent(upperLimit -> m_config.withSoftLimits(lowerLimit, upperLimit));
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
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismLimits(lower, upper);
      }
    });
  }

  @Override
  public void setMechanismLimitsEnabled(boolean enabled) {
    // The closed loop controller on the SystemCore enforces the mechanism limits.
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setMechanismLimitsEnabled(enabled);
      }
    });
  }

  @Override
  public void setMechanismGearing(MechanismGearing gearing) {
    m_config.withGearing(gearing);
    if (RobotBase.isSimulation()) {
      m_dcMotorSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(m_motor, m_config.getMOI(), gearing.getMechanismToRotorRatio()), m_motor));
      setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
    }
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

  @Override
  public void setClosedLoopSlot(ClosedLoopControllerSlot slot) {
    m_slot = slot;
    m_pid = m_config.getPID(slot);
    m_looseFollowers.ifPresent(smcs -> {
      for (var f : smcs) {
        f.setClosedLoopSlot(slot);
      }
    });
  }

  @Override
  public Temperature getTemperature() {
    return Celsius.of(m_nova.status().getTemperature());
  }

  @Override
  public SmartMotorControllerConfig<?> getConfig() {
    return m_config;
  }

  @Override
  public Object getMotorController() {
    return m_nova;
  }

  /**
   * {@inheritDoc}
   *
   * @return The vendor config given to {@link SmartMotorControllerConfig#withVendorConfig(Object)},
   *         or null without one. ThriftyLib configs are write-only actions; read the Nova's
   *         configuration back from {@code ((Nova) getMotorController()).status()}.
   */
  @Override
  public Object getMotorControllerConfig() {
    return m_novaVendorConfig.orElse(null);
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>> getUnsupportedTelemetryFields() {
    return Pair.of(Optional.empty(), Optional.empty());
  }
}
