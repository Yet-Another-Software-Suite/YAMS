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
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.A301;
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
import yams.core.motorcontrollers.YMotors;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.simulation.BatterySim;
import yams.core.motorcontrollers.simulation.DCMotorSimSupplier;
import yams.core.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.core.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * A301 wrapper for the FIRST A301 smart motor, using REVLib 2027.
 *
 * <p>The A301 has no closed loop gains, so the closed loop controller runs on the SystemCore and
 * commands the A301 with voltage. It also has no current limits, ramp rates, voltage compensation
 * or follower mode, and configuring them throws a {@link
 * SmartMotorControllerConfigurationException}.
 *
 * <p>The gearing given to {@link SmartMotorControllerConfig#withGearing} is from the A301's rotor
 * to the mechanism: usually the gearbox attached to the A301, and any gearing after it. Each A301
 * has one motor, {@link YMotors#getA301(int)}.
 *
 * <p><b>Several A301s on one mechanism</b>: give the main A301's wrapper how many A301s drive the
 * mechanism with {@link #A301Wrapper(A301, int, SmartMotorControllerConfig)}, so the simulation and
 * the mechanism's model have all of their motors. Wrap each of the other A301s in its own {@link
 * A301Wrapper} and give them to {@link
 * SmartMotorControllerConfig#withLooselyCoupledFollowers(SmartMotorController...)}, since the A301
 * has no follower mode.
 *
 * <p>REVLib does not simulate the A301, so in simulation the wrapper drives a {@link DCMotorSim}.
 *
 * <p><b>External encoder</b>: the A301's own absolute encoder, on its output shaft, used by giving
 * the {@link A301} itself to {@link SmartMotorControllerConfig#withExternalEncoder(Object)}. Its
 * gearing, given to {@link SmartMotorControllerConfig#withExternalEncoderGearing}, is from the
 * output shaft to the mechanism. Its zero offset, inversion and wrapping range are applied by the
 * wrapper; without {@link SmartMotorControllerConfig#withWrapping} it reads [0, 1) rotations of the
 * encoder.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // Configure and create an A301 on CAN bus 0, CAN ID 3, with its 64:1 gearbox
 * A301 a301 = new A301(CANPort.CAN_S0, 3);
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig()
 *     .withGearing(new MechanismGearing(GearBox.fromReductionStages(64)))
 *     .withMotorInverted(false)
 *     .withClosedLoopController(0.2, 0, 0)
 *     .withExternalEncoder(a301)
 *     .withUseExternalFeedbackEncoder(true);
 * SmartMotorController motor = new A301Wrapper(a301, config);
 * }</pre>
 */
public class A301Wrapper extends SmartMotorController {
  /** FIRST A301 smart motor. */
  private final A301 m_a301;

  /** Motors of the A301s on the mechanism. */
  private final DCMotor m_motor;

  /** Sim for the A301. */
  private Optional<DCMotorSim> m_dcMotorSim = Optional.empty();

  /** Whether the A301's absolute encoder is the external encoder. */
  private boolean m_useAbsoluteEncoder = false;

  /** Acceleration filter. */
  private final DerivativeTimeFilter m_accelerationFilter =
      new DerivativeTimeFilter(Milliseconds.of(20));

  /**
   * Duty cycle last applied to the simulated motor. The simulation holds it between commands, as
   * the A301 holds its output, so it must be what was commanded and not read back from the motor
   * model.
   */
  private volatile double m_simDutyCycle = 0;

  /**
   * Construct the A301 Wrapper for the generic {@link SmartMotorController}, for a mechanism driven
   * by this one A301.
   *
   * @param controller {@link A301} to use.
   * @param config {@link SmartMotorControllerConfig} to apply to the {@link A301}.
   * @throws SmartMotorControllerConfigurationException if {@link
   *     #applyConfig(SmartMotorControllerConfig)} rejects the config.
   * @throws IllegalArgumentException if {@link #applyConfig(SmartMotorControllerConfig)} rejects
   *     the config.
   */
  public A301Wrapper(A301 controller, SmartMotorControllerConfig<?> config) {
    this(controller, 1, config);
  }

  /**
   * Construct the A301 Wrapper for the generic {@link SmartMotorController}, for a mechanism driven
   * by several A301s. This is the main A301; wrap each of the others in its own {@link A301Wrapper}
   * and give them to {@link
   * SmartMotorControllerConfig#withLooselyCoupledFollowers(SmartMotorController...)}.
   *
   * @param controller Main {@link A301} of the mechanism.
   * @param motorCount How many A301s drive the mechanism, including this one. The simulation and
   *     {@link #getDCMotor()} model all of them.
   * @param config {@link SmartMotorControllerConfig} to apply to the {@link A301}.
   * @throws IllegalArgumentException if the motor count is less than 1, or if {@link
   *     #applyConfig(SmartMotorControllerConfig)} rejects the config.
   * @throws SmartMotorControllerConfigurationException if {@link
   *     #applyConfig(SmartMotorControllerConfig)} rejects the config.
   */
  public A301Wrapper(A301 controller, int motorCount, SmartMotorControllerConfig<?> config) {
    if (motorCount < 1) {
      throw new IllegalArgumentException(
          "A301Wrapper motor count must be at least 1, but was " + motorCount);
    }
    m_a301 = controller;
    m_motor = YMotors.getA301(motorCount);
    m_config = config;
    m_systemCoreClosedLoopAlert = Optional.of(new Alert("YAMS",
        buildAlertId("A301", m_a301.getDeviceId(), "ClosedLoop"),
        getName() + " closed loop controller is running on the SystemCore.", Alert.Level.MEDIUM));
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

  @Override
  public void setupSimulation() {
    if (RobotBase.isSimulation()) {
      if (m_dcMotorSim.isEmpty()) {
        var plant = Models.singleJointedArmFromPhysicalConstants(
            m_motor, m_config.getMOI(), m_config.getGearing().getMechanismToRotorRatio());
        m_dcMotorSim = Optional.of(new DCMotorSim(plant, m_motor));
        setSimSupplier(new DCMotorSimSupplier(m_dcMotorSim.get(), this));
      }
      m_config.getStartingPosition().ifPresent(
          startingPos -> m_simSupplier.ifPresent(sim -> sim.setMechanismPosition(startingPos)));
    }
  }

  /**
   * Get the angle of the A301's absolute encoder, after its zero offset, inversion and wrapping.
   *
   * @return Angle of the absolute encoder's shaft, or empty when it is not the external encoder.
   */
  private Optional<Angle> getExternalEncoderSensorPosition() {
    if (!m_useAbsoluteEncoder) {
      return Optional.empty();
    }
    final double rotations;
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      // REVLib does not simulate the A301, so the encoder follows the simulated mechanism.
      rotations = m_simSupplier.get()
                      .getMechanismPosition()
                      .times(m_config.getExternalEncoderGearing()
                              .orElse(MechanismGearing.kOne)
                              .getMechanismToRotorRatio())
                      .in(Rotations);
    } else {
      final double raw = m_a301.getAbsoluteEncoderPosition().get(0.0);
      rotations = (m_config.getExternalEncoderInverted().orElse(false) ? -raw : raw)
          - getZeroOffsetRotations();
    }
    // The absolute encoder reports angles in the wrapping range, or [0, 1) rotations.
    return Optional.of(Rotations.of(MathUtil.inputModulus(rotations,
        m_config.getWrappingMin().orElse(Rotations.zero()).in(Rotations),
        m_config.getWrappingMax().orElse(Rotations.of(1)).in(Rotations))));
  }

  /**
   * Zero offset of the absolute encoder, in rotations of the encoder.
   *
   * @return Zero offset in rotations of the encoder, 0 without one.
   */
  private double getZeroOffsetRotations() {
    return m_config.getExternalEncoderZeroOffset()
        .map(offset
            -> offset
                .times(m_config.getExternalEncoderGearing()
                        .orElse(MechanismGearing.kOne)
                        .getMechanismToRotorRatio())
                .in(Rotations))
        .orElse(0.0);
  }

  @Override
  public void seedRelativeEncoder() {
    getExternalEncoderMechanismPosition().ifPresent(mechanismAngle
        -> m_a301.setRelativeEncoderPosition(
            mechanismAngle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations)));
  }

  @Override
  public void synchronizeRelativeEncoder() {
    if (m_config.getFeedbackSynchronizationThreshold().isPresent()) {
      final Optional<Angle> externalEncoderAngle = getExternalEncoderMechanismPosition();
      if (externalEncoderAngle.isPresent()
          && !getRelativeMechanismPosition().isNear(
              externalEncoderAngle.get(), m_config.getFeedbackSynchronizationThreshold().get())) {
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
    }
  }

  @Override
  public void setZeroPower(MotorMode mode) {
    m_a301.setIdleMode(mode == MotorMode.BRAKE ? A301.IdleMode.kBrake : A301.IdleMode.kCoast);
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException if called outside of simulation, since the A301 does not
   *     support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(AngularVelocity velocity) {
    if (!RobotBase.isSimulation()) {
      throw new UnsupportedOperationException("A301 does not support setting encoder velocity.");
    }
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismVelocity(velocity));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
   * @throws UnsupportedOperationException if called outside of simulation, since the A301 does not
   *     support setting encoder velocity.
   */
  @Override
  public void setEncoderVelocity(LinearVelocity velocity) {
    setEncoderVelocity(m_config.convertToMechanism(velocity));
  }

  @Override
  public void setEncoderPosition(Angle angle) {
    if (m_useAbsoluteEncoder && !RobotBase.isSimulation()) {
      // Write the raw reading that the zero offset and inversion turn into this angle.
      final double encoderRotations = angle
                                          .times(m_config.getExternalEncoderGearing()
                                                  .orElse(MechanismGearing.kOne)
                                                  .getMechanismToRotorRatio())
                                          .in(Rotations)
          + getZeroOffsetRotations();
      m_a301.setAbsoluteEncoderPosition(m_config.getExternalEncoderInverted().orElse(false)
              ? -encoderRotations
              : encoderRotations);
    }
    m_a301.setRelativeEncoderPosition(
        angle.times(m_config.getGearing().getMechanismToRotorRatio()).in(Rotations));
    m_simSupplier.ifPresent(simSupplier -> simSupplier.setMechanismPosition(angle));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
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
   *     configured.
   */
  @Override
  public void setPosition(Distance distance) {
    setPosition(m_config.convertToMechanism(distance));
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
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
   * @throws SmartMotorControllerConfigurationException if a vendor config, current limit, ramp
   *     rate, voltage compensation, follower or vendor control request is configured, since the
   *     A301 has none of them.
   * @throws SmartMotorControllerConfigurationException if an external encoder option is configured
   *     without an external encoder.
   * @throws SmartMotorControllerConfigurationException if a required config option is left
   *     unhandled during validation.
   * @throws IllegalArgumentException if a closed loop control period is configured while not in
   *     closed loop mode.
   * @throws IllegalArgumentException if a closed loop tolerance is configured with an LQR
   *     controller.
   * @throws IllegalArgumentException if the external encoder is not this {@link A301}.
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

    if (config.getVendorConfig().isPresent()) {
      throw new SmartMotorControllerConfigurationException("A301 has no vendor config",
          "Vendor config could not be applied", ".withVendorConfig should be removed");
    }
    if (config.getVendorControlRequest().isPresent()) {
      throw new SmartMotorControllerConfigurationException(
          "A301(" + m_a301.getDeviceId() + ") does not support the custom control requests!",
          "Cannot use given control request", "withVendorControlRequest()");
    }
    config.getResetPreviousConfig();

    // Motion profiles and feedforwards run in the closed loop controller, on the SystemCore.
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
      throw new IllegalArgumentException(
          "[Error] Closed loop tolerance is not supported in LQR mode.");
    }
    config.getClosedLoopTolerance().ifPresent(tolerance -> {
      if (config.getLinearClosedLoopControllerUse()) {
        m_pid.ifPresent(pidController
            -> pidController.setTolerance(config.convertFromMechanism(tolerance).in(Meters)));
      } else {
        m_pid.ifPresent(pidController -> pidController.setTolerance(tolerance.in(Rotations)));
      }
    });

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

    // The A301 has no ramp rates, current limits or voltage compensation.
    if (config.getOpenLoopRampRate().isPresent() || config.getClosedLoopRampRate().isPresent()) {
      throw new SmartMotorControllerConfigurationException("A301 does not support ramp rates",
          "Ramp rate could not be applied",
          ".withOpenLoopRampRate and .withClosedLoopRampRate should be removed");
    }
    if (config.getStatorStallCurrentLimit().isPresent()
        || config.getSupplyStallCurrentLimit().isPresent()) {
      throw new SmartMotorControllerConfigurationException("A301 does not support current limits",
          "Current limit could not be applied",
          ".withStatorCurrentLimit and .withSupplyCurrentLimit should be removed");
    }
    if (config.getVoltageCompensation().isPresent()) {
      throw new SmartMotorControllerConfigurationException(
          "A301 does not support voltage compensation", "Voltage compensation could not be applied",
          ".withVoltageCompensation should be removed");
    }

    config.getMotorInverted().ifPresent(m_a301::setInverted);
    if (config.getEncoderInverted().isPresent()) {
      throw new IllegalArgumentException("[ERROR] A301 internal encoder cannot be inverted!");
    }
    config.getZeroPower().ifPresent(this::setZeroPower);

    // Wrapping sets the absolute encoder's range; continuous wrapping is handled by the closed loop
    // controller. Read the maximum first, since it validates the range.
    config.getWrappingMax();
    config.getWrappingMin();
    config.isWrappingContinuous();

    // External encoder: the A301's absolute encoder.
    m_useAbsoluteEncoder = false;
    config.getUseExternalFeedback();
    if (config.getExternalEncoder().isPresent()) {
      final Object externalEncoder = config.getExternalEncoder().get();
      if (externalEncoder != m_a301) {
        throw new IllegalArgumentException(
            "[ERROR] Unsupported external encoder: " + externalEncoder.getClass().getSimpleName()
            + ".\n\tPlease use the A301 itself, for its absolute encoder, instead.");
      }
      m_useAbsoluteEncoder = true;
      config.getExternalEncoderGearing();
      config.getExternalEncoderInverted();
      config.getExternalEncoderZeroOffset();
    } else {
      if (config.getExternalEncoderZeroOffset().isPresent()) {
        throw new SmartMotorControllerConfigurationException(
            "Zero offset is only available for external encoders",
            "Zero offset could not be applied", ".withExternalEncoderZeroOffset");
      }
      if (config.getExternalEncoderInverted().isPresent()) {
        throw new SmartMotorControllerConfigurationException(
            "External encoder cannot be inverted because no external encoder exists",
            "External encoder could not be inverted", ".withExternalEncoderInverted");
      }
      if (config.getExternalEncoderGearing().isPresent()) {
        throw new SmartMotorControllerConfigurationException(
            "External encoder gearing is not supported when there is no external encoder",
            "External encoder gearing could not be set", ".withExternalEncoderGearing");
      }
    }

    // Starting position, or the absolute encoder's position without one.
    if (config.getStartingPosition().isPresent()) {
      m_a301.setRelativeEncoderPosition(config.getStartingPosition()
              .get()
              .times(config.getGearing().getMechanismToRotorRatio())
              .in(Rotations));
      m_simSupplier.ifPresent(sim -> sim.setMechanismPosition(config.getStartingPosition().get()));
    } else if (m_useAbsoluteEncoder) {
      seedRelativeEncoder();
    }

    if (config.getFollowers().isPresent()) {
      throw new SmartMotorControllerConfigurationException(
          "A301 cannot be followed by another motor controller", "Followers could not be applied",
          ".withLooselyCoupledFollowers instead of .withFollowers");
    }

    if (config.getMotorControllerMode() == ControlMode.CLOSED_LOOP) {
      m_systemCoreClosedLoopAlert.ifPresent(alert -> alert.set(true));
      startClosedLoopController();
    } else if (config.getClosedLoopControlPeriod().isPresent()) {
      throw new IllegalArgumentException(
          "[Error] Closed loop control period is only supported in closed loop mode.");
    }

    config.validateBasicOptions();
    config.validateExternalEncoderOptions();
    return true;
  }

  @Override
  public double getDutyCycle() {
    return m_simSupplier.isPresent() ? m_simDutyCycle : m_a301.getAppliedOutput().get(0.0);
  }

  @Override
  public void setDutyCycle(double dutyCycle) {
    m_a301.setThrottle(dutyCycle);
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

  /**
   * {@inheritDoc}
   *
   * @return The simulated supply current, or empty on the robot, since the A301 does not report
   *     it.
   */
  @Override
  public Optional<Current> getSupplyCurrent() {
    return m_simSupplier.map(simSupplier -> simSupplier.getSupplyCurrent());
  }

  @Override
  public Current getStatorCurrent() {
    return m_simSupplier.map(simSupplier -> simSupplier.getStatorCurrent())
        .orElseGet(() -> Amps.of(m_a301.getMotorCurrent().get(0.0)));
  }

  @Override
  public Voltage getVoltage() {
    return m_simSupplier
        .map(simSupplier -> simSupplier.getMechanismSupplyVoltage().times(m_simDutyCycle))
        .orElseGet(
            () -> Volts.of(m_a301.getAppliedOutput().get(0.0) * m_a301.getBusVoltage().get(0.0)));
  }

  @Override
  public void setVoltage(Voltage voltage) {
    m_a301.setVoltage(voltage.in(Volts));
    m_simSupplier.ifPresent(simSupplier -> {
      // The A301 cannot apply more than its supply voltage.
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
   *     configured.
   */
  @Override
  public LinearVelocity getMeasurementVelocity() {
    return m_config.convertFromMechanism(getMechanismVelocity());
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
   */
  @Override
  public Distance getMeasurementPosition() {
    return m_config.convertFromMechanism(getMechanismPosition());
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
   */
  @Override
  public LinearAcceleration getMeasurementAcceleration() {
    return m_config.convertFromMechanism(getMechanismAcceleration());
  }

  /**
   * {@inheritDoc}
   *
   * <p>The A301 does not report its absolute encoder's velocity, so this is the A301's relative
   * encoder's velocity at the mechanism.
   */
  @Override
  public AngularVelocity getMechanismVelocity() {
    return getRelativeMechanismVelocity();
  }

  @Override
  public AngularAcceleration getMechanismAcceleration() {
    return RotationsPerSecond.per(Microsecond)
        .of(m_accelerationFilter.derivative(getMechanismVelocity().in(RotationsPerSecond)));
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
    return RPM.of(m_a301.getEncoderVelocity().get(0.0));
  }

  @Override
  public Angle getRotorPosition() {
    if (RobotBase.isSimulation() && m_simSupplier.isPresent()) {
      return m_simSupplier.get().getRotorPosition();
    }
    return Rotations.of(m_a301.getRelativeEncoderPosition().get(0.0));
  }

  @Override
  public Optional<Angle> getExternalEncoderMechanismPosition() {
    return getExternalEncoderSensorPosition().map(angle
        -> angle.times(m_config.getExternalEncoderGearing()
                .orElse(MechanismGearing.kOne)
                .getRotorToMechanismRatio()));
  }

  /**
   * {@inheritDoc}
   *
   * <p>The A301 does not report its absolute encoder's velocity, so this is the A301's relative
   * encoder's velocity at the mechanism.
   */
  @Override
  public Optional<AngularVelocity> getExternalEncoderMechanismVelocity() {
    return m_useAbsoluteEncoder ? Optional.of(getRelativeMechanismVelocity()) : Optional.empty();
  }

  @Override
  public void setMotorInverted(boolean inverted) {
    m_config.withMotorInverted(inverted);
    m_a301.setInverted(inverted);
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the A301's internal encoder cannot be
   *     inverted.
   */
  @Override
  public void setEncoderInverted(boolean inverted) {
    throw new UnsupportedOperationException("A301 internal encoder cannot be inverted.");
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if the mechanism circumference is not
   *     configured.
   */
  @Override
  public void setMotionProfileMaxVelocity(LinearVelocity maxVelocity) {
    if (m_trapezoidProfile.isPresent()) {
      final Constraints constraints = new Constraints(maxVelocity.in(MetersPerSecond),
          m_config.getTrapezoidProfile().orElseThrow().maxAcceleration);
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
   *     configured.
   */
  @Override
  public void setMotionProfileMaxAcceleration(LinearAcceleration maxAcceleration) {
    if (m_trapezoidProfile.isPresent()) {
      final Constraints constraints =
          new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity,
              maxAcceleration.in(MetersPerSecondPerSecond));
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
      final Constraints constraints = new Constraints(maxVelocity.in(RotationsPerSecond),
          m_config.getTrapezoidProfile().orElseThrow().maxAcceleration);
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
      final Constraints constraints =
          new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity,
              maxAcceleration.in(RotationsPerSecondPerSecond));
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
      final Constraints constraints =
          new Constraints(m_config.getTrapezoidProfile().orElseThrow().maxVelocity,
              maxJerk.in(RotationsPerSecondPerSecond.per(Second)));
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
  public void setExponentialProfile(
      OptionalDouble kV, OptionalDouble kA, Optional<Voltage> maxInput) {
    if (m_expoProfile.isPresent() && m_config.getExponentialProfile().isPresent()) {
      var exp = m_config.getExponentialProfile().get();
      // kV and kA are in the profile's units, like its constraints.
      var defaultkV = -exp.A / exp.B;
      var defaultkA = 1.0 / exp.B;
      var defaultMaxInput = exp.maxInput;
      final ExponentialProfile.Constraints constraints =
          ExponentialProfile.Constraints.fromCharacteristics(
              maxInput.orElse(Volts.of(defaultMaxInput)).in(Volts), kV.orElse(defaultkV),
              kA.orElse(defaultkA));
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

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the A301 has no current limits.
   */
  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    throw new UnsupportedOperationException("A301 does not support current limits.");
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the A301 has no current limits.
   */
  @Override
  public void setSupplyCurrentLimit(Current currentLimit) {
    throw new UnsupportedOperationException("A301 does not support current limits.");
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the A301 has no ramp rates.
   */
  @Override
  public void setClosedLoopRampRate(Time rampRate) {
    throw new UnsupportedOperationException("A301 does not support ramp rates.");
  }

  /**
   * {@inheritDoc}
   *
   * @throws UnsupportedOperationException always, since the A301 has no ramp rates.
   */
  @Override
  public void setOpenLoopRampRate(Time rampRate) {
    throw new UnsupportedOperationException("A301 does not support ramp rates.");
  }

  /**
   * {@inheritDoc}
   *
   * @throws SmartMotorControllerConfigurationException if a lower limit is configured and it is
   *     greater than or equal to the new upper limit.
   */
  @Override
  public void setMeasurementUpperLimit(Distance upperLimit) {
    if (m_config.getMechanismCircumference().isPresent()
        && m_config.getMechanismLowerLimit().isPresent()) {
      m_config.withSoftLimits(
          m_config.convertFromMechanism(m_config.getMechanismLowerLimit().get()), upperLimit);
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
   *     lower limit is greater than or equal to it.
   */
  @Override
  public void setMeasurementLowerLimit(Distance lowerLimit) {
    if (m_config.getMechanismCircumference().isPresent()
        && m_config.getMechanismUpperLimit().isPresent()) {
      m_config.withSoftLimits(
          lowerLimit, m_config.convertFromMechanism(m_config.getMechanismUpperLimit().get()));
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
   *     greater than or equal to the new upper limit.
   */
  @Override
  public void setMechanismUpperLimit(Angle upperLimit) {
    m_config.getMechanismLowerLimit().ifPresent(
        lowerLimit -> m_config.withSoftLimits(lowerLimit, upperLimit));
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
   *     lower limit is greater than or equal to it.
   */
  @Override
  public void setMechanismLowerLimit(Angle lowerLimit) {
    m_config.getMechanismUpperLimit().ifPresent(
        upperLimit -> m_config.withSoftLimits(lowerLimit, upperLimit));
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
   *     limit is greater than or equal to the upper limit.
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
      // The simulated motor's reduction comes from the gearing.
      var plant = Models.singleJointedArmFromPhysicalConstants(
          m_motor, m_config.getMOI(), gearing.getMechanismToRotorRatio());
      m_dcMotorSim = Optional.of(new DCMotorSim(plant, m_motor));
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
    return Celsius.of(m_a301.getMotorTemperature().get(0.0));
  }

  @Override
  public SmartMotorControllerConfig<?> getConfig() {
    return m_config;
  }

  @Override
  public Object getMotorController() {
    return m_a301;
  }

  /**
   * {@inheritDoc}
   *
   * @return null, since the A301 has no vendor config.
   */
  @Override
  public Object getMotorControllerConfig() {
    return null;
  }

  @Override
  public Pair<Optional<List<BooleanTelemetryField>>, Optional<List<DoubleTelemetryField>>>
  getUnsupportedTelemetryFields() {
    return Pair.of(Optional.empty(),
        Optional.of(List.of(DoubleTelemetryField.SupplyCurrent,
            DoubleTelemetryField.SupplyCurrentLimit, DoubleTelemetryField.StatorCurrentLimit,
            DoubleTelemetryField.ClosedloopRampRate, DoubleTelemetryField.OpenloopRampRate)));
  }
}
