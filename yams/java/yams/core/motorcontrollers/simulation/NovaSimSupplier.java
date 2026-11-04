// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.thrifty.core.Motor;
import com.thrifty.core.MotorConfig.MPGravityType;
import com.thrifty.core.MotorControl.ControlType;
import com.thrifty.core.MotorControlRequest;
import com.thrifty.nova.Nova;
import java.util.Optional;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import yams.core.motorcontrollers.SimSupplier;

/**
 * A simulated Thrifty Nova driving a mechanism's {@link SimSupplier}. ThriftyLib's simulation
 * ignores the Nova's closed loop, so this runs it as the Nova would, with the gains read back from
 * the Nova: the voltage position and velocity loops with the PID slot's gains and its F times the
 * target, and the motion profile with its slot's kS, kV, kA and kG. It keeps within the Nova's soft
 * limits, and, with its absolute wrapping on, takes a position the short way, one mechanism rotation
 * apart. Targets and gains are in mechanism rotations, as on a Nova with its
 * sensor-to-mechanism ratio set, and the integral and derivative gains are per second, as the
 * Nova's are. Open loop output, a duty cycle or a voltage, is passed on to the mechanism.
 */
public class NovaSimSupplier implements SimSupplier {
  /** Mechanism the Nova drives. */
  private final SimSupplier mechanism;
  /** Nova whose gains are simulated. */
  private final Nova nova;
  /** Simulation period, in seconds. */
  private final double periodSeconds;
  /** Closed loop request being run, or empty in open loop. */
  private Optional<MotorControlRequest> request = Optional.empty();
  /** Duty cycle applied to the motor. */
  private double dutyCycle = 0;
  /** Sum of the closed loop error over every Nova loop. */
  private double integral = 0;
  /** Closed loop error of the last period, or empty after a new type of request. */
  private Optional<Double> lastError = Optional.empty();
  /** Point the motion profile reached, or empty before it starts. */
  private Optional<TrapezoidProfile.State> profileState = Optional.empty();

  /**
   * Create a simulated Nova.
   *
   * @param mechanism Mechanism the Nova drives.
   * @param nova      Nova whose gains are simulated.
   * @param period    Simulation period.
   */
  public NovaSimSupplier(SimSupplier mechanism, Nova nova, Time period) {
    this.mechanism = mechanism;
    this.nova = nova;
    this.periodSeconds = period.in(Seconds);
  }

  /**
   * Run a closed loop control request, as the Nova does when it is sent one. The integral, and the
   * motion profile, carry on while requests of the same type are sent.
   *
   * @param controlRequest Voltage position, voltage velocity or motion profile request.
   */
  public void setControlRequest(MotorControlRequest controlRequest) {
    if (request.isEmpty() || request.get().getControlType() != controlRequest.getControlType()) {
      integral = 0;
      lastError = Optional.empty();
      profileState = Optional.empty();
    }
    request = Optional.of(controlRequest);
  }

  /**
   * Get the closed loop request being run.
   *
   * @return The request, or empty in open loop.
   */
  public Optional<MotorControlRequest> getControlRequest() {
    return request;
  }

  /**
   * Get the duty cycle applied to the motor.
   *
   * @return Duty cycle, from -1 to 1.
   */
  public double getDutyCycle() {
    return dutyCycle;
  }

  /**
   * Apply a voltage to the motor, within the supply voltage.
   *
   * @param volts Voltage to apply.
   */
  private void applyVoltage(double volts) {
    // Past a soft limit, the Nova does not drive further that way.
    final var status = nova.status();
    final double position = mechanism.getMechanismPosition().in(Rotations);
    if (status.getSoftLimitEnable() && ((position >= status.getSoftLimitForward() && volts > 0) || (position <= status.getSoftLimitReverse() && volts < 0))) {
      volts = 0;
    }
    final double supplyVolts = mechanism.getMechanismSupplyVoltage().in(Volts);
    dutyCycle = Math.clamp(volts / supplyVolts, -1, 1);
    mechanism.setMechanismStatorVoltage(Volts.of(supplyVolts * dutyCycle));
  }

  @Override
  public void updateSimState() {
    request.ifPresent(controlRequest -> {
      final boolean slot1 = controlRequest.getSlot() == Motor.Slot.SLOT1;
      final var status = nova.status();
      final double position = mechanism.getMechanismPosition().in(Rotations);
      final double velocity = mechanism.getMechanismVelocity().in(RotationsPerSecond);
      // The request's feedforward, and the PID slot's F times the target.
      double feedforward = controlRequest.getFF() + (slot1 ? status.Pid1.getF() : status.Pid0.getF()) * controlRequest.getTarget();
      // With continuous wrapping, a position target is the equivalent nearest the mechanism, one
      // mechanism rotation apart.
      final double positionTarget = status.getAbsWrapping()
                                    ? position + MathUtil.inputModulus(controlRequest.getTarget() - position, -0.5, 0.5)
                                    : controlRequest.getTarget();
      double error;
      if (controlRequest.getControlType() == ControlType.VOLTAGE_VELOCITY) {
        error = controlRequest.getTarget() - velocity;
      } else if (controlRequest.getControlType() == ControlType.MOTION_PROFILE) {
        final double kS = slot1 ? status.Mp1.getKS() : status.Mp0.getKS();
        final double kV = slot1 ? status.Mp1.getKV() : status.Mp0.getKV();
        final double kA = slot1 ? status.Mp1.getKA() : status.Mp0.getKA();
        final double kG = slot1 ? status.Mp1.getKG() : status.Mp0.getKG();
        final MPGravityType gravityType = slot1 ? status.Mp1.getGravType() : status.Mp0.getGravType();
        final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(slot1 ? status.Mp1.getMaxVel() : status.Mp0.getMaxVel(),
                                                                                               slot1 ? status.Mp1.getMaxAccel() : status.Mp0.getMaxAccel()));
        final TrapezoidProfile.State state = profileState.orElseGet(() -> new TrapezoidProfile.State(position, velocity));
        final TrapezoidProfile.State next = profile.calculate(periodSeconds, state, new TrapezoidProfile.State(positionTarget, 0));
        profileState = Optional.of(next);
        // kG is the gravity with the mechanism at 0, horizontal for an arm.
        feedforward += kS * Math.signum(next.velocity) + kV * next.velocity + kA * (next.velocity - state.velocity) / periodSeconds
                       + kG * (gravityType == MPGravityType.COSINE ? Math.cos(2 * Math.PI * position) : 1);
        error = next.position - position;
      } else {
        error = positionTarget - position;
      }
      if (Math.abs(error) <= (slot1 ? status.Pid1.getAllowableError() : status.Pid0.getAllowableError())) {
        error = 0;
      }
      // The integral and the rate of change of the error, per second.
      integral += error * periodSeconds;
      final double derivative = (error - lastError.orElse(error)) / periodSeconds;
      lastError = Optional.of(error);
      applyVoltage((slot1 ? status.Pid1.getP() : status.Pid0.getP()) * error
                   + (slot1 ? status.Pid1.getI() : status.Pid0.getI()) * integral
                   + (slot1 ? status.Pid1.getD() : status.Pid0.getD()) * derivative
                   + feedforward);
    });
    mechanism.updateSimState();
  }

  @Override
  public boolean getUpdatedSim() {
    return mechanism.getUpdatedSim();
  }

  @Override
  public void feedUpdateSim() {
    mechanism.feedUpdateSim();
  }

  @Override
  public void starveUpdateSim() {
    mechanism.starveUpdateSim();
  }

  @Override
  public boolean isInputFed() {
    return mechanism.isInputFed();
  }

  @Override
  public void feedInput() {
    mechanism.feedInput();
  }

  @Override
  public void starveInput() {
    mechanism.starveInput();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Open loop: the Nova stops its closed loop.
   */
  @Override
  public void setMechanismStatorDutyCycle(double dutyCycle) {
    request = Optional.empty();
    applyVoltage(dutyCycle * mechanism.getMechanismSupplyVoltage().in(Volts));
  }

  @Override
  public Voltage getMechanismSupplyVoltage() {
    return mechanism.getMechanismSupplyVoltage();
  }

  @Override
  public Voltage getMechanismStatorVoltage() {
    return mechanism.getMechanismStatorVoltage();
  }

  /**
   * {@inheritDoc}
   *
   * <p>Open loop: the Nova stops its closed loop.
   */
  @Override
  public void setMechanismStatorVoltage(Voltage volts) {
    request = Optional.empty();
    applyVoltage(volts.in(Volts));
  }

  @Override
  public Angle getMechanismPosition() {
    return mechanism.getMechanismPosition();
  }

  @Override
  public void setMechanismPosition(Angle position) {
    profileState = Optional.empty();
    mechanism.setMechanismPosition(position);
  }

  @Override
  public Angle getRotorPosition() {
    return mechanism.getRotorPosition();
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return mechanism.getMechanismVelocity();
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity) {
    profileState = Optional.empty();
    mechanism.setMechanismVelocity(velocity);
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return mechanism.getRotorVelocity();
  }

  @Override
  public Current getStatorCurrent() {
    return mechanism.getStatorCurrent();
  }

  @Override
  public Current getSupplyCurrent() {
    return mechanism.getSupplyCurrent();
  }

  @Override
  public AngularAcceleration getRotorAcceleration() {
    return mechanism.getRotorAcceleration();
  }
}
