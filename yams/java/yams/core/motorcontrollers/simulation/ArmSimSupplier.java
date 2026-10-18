// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.simulation;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.KilogramSquareMeters;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Microsecond;
import static org.wpilib.units.Units.Milliseconds;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import java.util.Optional;
import java.util.UUID;
import java.util.function.Supplier;
import org.wpilib.math.filter.LinearFilter;
import org.wpilib.math.system.DCMotor;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularAcceleration;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.Mass;
import org.wpilib.units.measure.MomentOfInertia;
import org.wpilib.units.measure.Time;
import org.wpilib.units.measure.Voltage;
import yams.core.gearing.MechanismGearing;
import yams.core.math.DerivativeTimeFilter;
import yams.core.motorcontrollers.SimSupplier;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * ArmSim Supplier simulates a single-jointed arm mechanism using WPILib's
 * {@link org.wpilib.simulation.SingleJointedArmSim}.
 *
 * <p>This supplier steps WPILib's {@code SingleJointedArmSim} physics model each control loop and
 * exposes the resulting angle, angular velocity, current draw, and voltage through the
 * {@link yams.core.motorcontrollers.SimSupplier} interface. The arm's gear ratio and control period are
 * read directly from the associated {@link yams.core.motorcontrollers.SmartMotorController}'s
 * config, so no duplication of parameters is required.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * // 1. Build the WPILib arm physics model
 * SingleJointedArmSim armPhysics = new SingleJointedArmSim(
 *     DCMotor.getNEO(1),
 *     SingleJointedArmSim.estimateMOI(0.5, 2.0), // moment of inertia (kg·m²)
 *     5.0,                                         // gear ratio (rotor/mechanism)
 *     0.5,                                         // arm length (meters)
 *     Units.degreesToRadians(-10),                 // min angle (radians)
 *     Units.degreesToRadians(90),                  // max angle (radians)
 *     true,                                        // simulate gravity
 *     0);                                          // starting angle (radians)
 *
 * // 2. Configure and build the YAMS smart motor controller
 * SmartMotorController motor = new SparkMaxController(
 *     new SmartMotorControllerConfig()
 *         .withGearing(new MechanismGearing(5.0))
 *         .withClosedLoopControlPeriod(Milliseconds.of(20)));
 *
 * // 3. Wrap physics model in the supplier and register it
 * ArmSimSupplier sim = new ArmSimSupplier(armPhysics, motor);
 * motor.getConfig().withSimSupplier(sim);
 * }</pre>
 */
public class ArmSimSupplier implements SimSupplier {
  private boolean                    inputFed   = false;
  private boolean                    simUpdated = false;
  private final Supplier<Double>     motorDutyCycleSupplier;
  private final DerivativeTimeFilter accel;
  private final LinearFilter         supplyCurrentFilter;
  private final SingleJointedArmSim  sim;
  private final MechanismGearing     mechGearing;
  private final Time                 simPeriod;
  private final DCMotor              motor;
  private final UUID                 uuid;
  /** The game pieces the arm carries. */
  private final GamePieces           gamePieces = new GamePieces();
  /** The arm's moment of inertia without game pieces, in kilogram square meters, if known. */
  private final Optional<Double>     armMoi;
  /** The arm's length, in meters, if known. */
  private final Optional<Double>     armLength;
  /** Whether the simulation simulates gravity. */
  private final boolean              simulateGravity;

  /**
   * Construct the ArmSim supplier. It cannot simulate game pieces, as it does not know the arm's moment
   * of inertia and length: use
   * {@link #ArmSimSupplier(SingleJointedArmSim, SmartMotorController, MomentOfInertia, Distance, boolean)}.
   *
   * @param simulation           Simulatoin instance
   * @param smartMotorController SMC for the ArmSim..
   */
  public ArmSimSupplier(SingleJointedArmSim simulation, SmartMotorController smartMotorController) {
    this(simulation, smartMotorController, Optional.empty(), Optional.empty(), true);
  }

  /**
   * Construct the ArmSim supplier, which can simulate game pieces.
   *
   * @param simulation           Simulation instance.
   * @param smartMotorController SMC for the ArmSim.
   * @param moi                  The arm's moment of inertia, as the simulation was given it.
   * @param length               The arm's length, as the simulation was given it.
   * @param simulateGravity      Whether the simulation simulates gravity, as it was given it.
   */
  public ArmSimSupplier(SingleJointedArmSim simulation, SmartMotorController smartMotorController, MomentOfInertia moi,
                        Distance length, boolean simulateGravity) {
    this(simulation, smartMotorController, Optional.of(moi.in(KilogramSquareMeters)), Optional.of(length.in(Meters)),
         simulateGravity);
  }

  private ArmSimSupplier(SingleJointedArmSim simulation, SmartMotorController smartMotorController,
                         Optional<Double> moi, Optional<Double> length, boolean simulateGravity) {
    armMoi = moi;
    armLength = length;
    this.simulateGravity = simulateGravity;
    var config = smartMotorController.getConfig();
    sim = simulation;
    motorDutyCycleSupplier = smartMotorController::getDutyCycle;
    mechGearing = config.getGearing();
    simPeriod = config.getSimulationPeriod();
    motor = smartMotorController.getDCMotor();
    accel = new DerivativeTimeFilter(simPeriod);
    // Based off comment from https://github.com/wpilibsuite/allwpilib/issues/8691
    supplyCurrentFilter = LinearFilter.singlePoleIIR(Milliseconds.of(100).in(Seconds), simPeriod.in(Seconds));
    uuid = smartMotorController.m_batterySimUUID;
  }

  @Override
  public void updateSimState() {
    if (!isInputFed()) {
      sim.setInputVoltage(motorDutyCycleSupplier.get() * RoboRioSim.getVInVoltage());
      RoboRioSim.setVInVoltage(BatterySim.calculateVoltage(uuid, getSupplyCurrent()));
    }
    if (!simUpdated) {
      starveInput();
      // Carrying game pieces, the simulation is given the voltage that moves the arm without them as
      // they move it, for this step, and then the motor's voltage again.
      final double volts = sim.getInput(0);
      final double simulatedVolts = gamePieces.isEmpty() ? volts : gamePieces.armVoltage(
          volts, getRotorVelocity().in(RadiansPerSecond) / motor.Kv, sim.getAngle(), armMoi.orElseThrow(),
          armLength.orElseThrow(), simulateGravity, motor, mechGearing.getMechanismToRotorRatio());
      sim.setInput(simulatedVolts);
      sim.update(simPeriod.in(Seconds));
      // Unless a closed loop on another thread gave the motor a new voltage meanwhile.
      if (sim.getInput(0) == simulatedVolts) {
        sim.setInput(volts);
      }
      try {
        // Thread.sleep(1);
      } catch (Exception e) {
      }
      feedUpdateSim();
    }
  }

  @Override
  public boolean getUpdatedSim() {
    return simUpdated;
  }

  @Override
  public void feedUpdateSim() {
    simUpdated = true;
  }

  @Override
  public void starveUpdateSim() {
    simUpdated = false;
  }

  @Override
  public boolean isInputFed() {
    return inputFed;
  }

  @Override
  public void feedInput() {
    inputFed = true;
  }

  @Override
  public void starveInput() {
    inputFed = false;
  }

  @Override
  public void setMechanismStatorDutyCycle(double dutyCycle) {
    feedInput();
    sim.setInputVoltage(dutyCycle * getMechanismSupplyVoltage().in(Volts));
  }

  @Override
  public Voltage getMechanismSupplyVoltage() {
    return Volts.of(RoboRioSim.getVInVoltage());
  }

  @Override
  public Voltage getMechanismStatorVoltage() {
    // The voltage the simulated motor was given. Worked out from the mechanism's torque or speed
    // instead, it would be off by the gear ratio, since the motor's equation is for the rotor.
    return Volts.of(sim.getInput(0));
  }

  @Override
  public void setMechanismStatorVoltage(Voltage volts) {
    feedInput();
    sim.setInputVoltage(volts.in(Volts));
  }

  @Override
  public Angle getMechanismPosition() {
    return Radians.of(sim.getAngle());
  }

  @Override
  public void setMechanismPosition(Angle position) {
    sim.setState(position.in(Radians), sim.getVelocity()); // .times(config.getGearing().getMechanismToRotorRatio()).in(Radians));
  }

  @Override
  public Angle getRotorPosition() {
    return getMechanismPosition().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public AngularVelocity getMechanismVelocity() {
    return RadiansPerSecond.of(sim.getVelocity());
  }

  @Override
  public void setMechanismVelocity(AngularVelocity velocity) {
    sim.setState(sim.getAngle(), velocity.in(RadiansPerSecond));
  }

  @Override
  public AngularVelocity getRotorVelocity() {
    return getMechanismVelocity().times(mechGearing.getMechanismToRotorRatio());
  }

  @Override
  public Current getStatorCurrent() {
    return Amps.of(sim.getCurrentDraw());
  }

  @Override
  public Current getSupplyCurrent() {
    // For a BLDC driven by a switching converter, power is conserved across the duty-cycle
    // transformation: supplyVoltage * supplyCurrent = statorVoltage * statorCurrent, and
    // statorVoltage = dutyCycle * supplyVoltage, so supplyCurrent = dutyCycle * statorCurrent.
    double dutyCycle = motorDutyCycleSupplier.get();
    return Amps.of(supplyCurrentFilter.calculate(dutyCycle * sim.getCurrentDraw()));
  }

  /**
   * {@inheritDoc}
   *
   * <p>The game piece is at the end of the arm.
   */
  @Override
  public void addGamePiece(Mass mass) {
    addGamePiece(mass, Meters.of(armLength.orElseThrow(this::cannotCarry)));
  }

  @Override
  public void addGamePiece(Mass mass, Distance distance) {
    if (armMoi.isEmpty() || armLength.isEmpty()) {
      throw cannotCarry();
    }
    gamePieces.add(mass, distance);
  }

  @Override
  public void removeGamePiece() {
    gamePieces.removeLast();
  }

  private UnsupportedOperationException cannotCarry() {
    return new UnsupportedOperationException("This ArmSimSupplier cannot simulate a game piece, as it was not given the arm's moment of inertia and length.");
  }

  @Override
  public AngularAcceleration getRotorAcceleration() {
    return RotationsPerSecond.per(Microsecond).of(accel.derivative(getRotorVelocity().in(RotationsPerSecond)));
  }
}
