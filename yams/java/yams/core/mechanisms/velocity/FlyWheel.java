// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.velocity;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.RPM;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;
import yams.core.mechanisms.config.FlyWheelConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * FlyWheel mechanism.
 *
 * <p>This core class holds flywheel state, physics simulation, and measurement/predicate logic
 * only. Command and Trigger factories live on {@link yams.commands2.mechanisms.FlyWheel}, which
 * extends this class and has a full usage example in its Javadoc.
 *
 * <p>Velocity mechanisms have no {@code max()}/{@code min()} concept; use
 * {@link #isNear(org.wpilib.units.measure.AngularVelocity, org.wpilib.units.measure.AngularVelocity)},
 * {@link #isGte(org.wpilib.units.measure.AngularVelocity)}, or
 * {@link #isLte(org.wpilib.units.measure.AngularVelocity)} instead.
 */
public class FlyWheel extends SmartVelocityMechanism {
  /**
   * FlyWheel config.
   */
  private final FlyWheelConfig m_config;
  /**
   * Simulation for the FlyWheel.
   */
  private Optional<DCMotorSim> m_dcmotorSim = Optional.empty();

  /**
   * Construct the FlyWheel class
   *
   * @param config FlyWheel configuration.
   * @param smc    {@link SmartMotorController} for the Mechanism
   * @implNote Protected so only {@link yams.commands2.mechanisms.FlyWheel} can construct this.
   */
  protected FlyWheel(FlyWheelConfig config, SmartMotorController smc) {
    m_config = config;
    m_smc = smc;
    SmartMotorControllerConfig smcCfg = smc.getConfig();
    DCMotor dcMotor = m_smc.getDCMotor();
    // Seed the relative encoder
    if (m_smc.getConfig().getExternalEncoder().isPresent()) {
      m_smc.seedRelativeEncoder();
    }
    if (config.getTelemetryName().isPresent()) {
      // TODO: Add telemetry units to config.
      m_telemetry.setupTelemetry(getName(), m_smc);
    }

    if (RobotBase.isSimulation()) {
      m_dcmotorSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(dcMotor, smcCfg.getMOI(), smcCfg.getGearing().getMechanismToRotorRatio()), dcMotor));

      m_smc.setSimSupplier(new DCMotorSimSupplier(m_dcmotorSim.get(), m_smc));
      Distance ShooterLength = config.getDiameter().orElse(Inches.of(36));
      m_mechanismWindow = new Mechanism2d(ShooterLength.in(Meters) * 2, ShooterLength.in(Meters) * 2);
      mechanismRoot = m_mechanismWindow.getRoot(getName() + "Root", ShooterLength.in(Meters), ShooterLength.in(Meters));
      mechanismLigament = mechanismRoot.append(new MechanismLigament2d(getName(), ShooterLength.in(Meters), 0, 6, config.getSimColor()));
      publishMechanismWindow();
    }
  }

  /**
   * Between two velocities.
   *
   * @param start Start Velocity.
   * @param end   End velocity
   * @return True if the FlyWheel's speed is between the given velocities.
   */
  public boolean isBetween(AngularVelocity start, AngularVelocity end) {
    return isGte(start) && isLte(end);
  }

  /**
   * Greater than or equal to angular velocity.
   *
   * @param speed {@link AngularVelocity} to check against.
   * @return True if the FlyWheel's speed is greater than or equal to the given speed.
   */
  public boolean isGte(AngularVelocity speed) {
    return getSpeed().gte(speed);
  }

  /**
   * Less than or equal to angular velocity
   *
   * @param speed {@link AngularVelocity} to check against
   * @return True if the FlyWheel's speed is less than or equal to the given speed.
   */
  public boolean isLte(AngularVelocity speed) {
    return getSpeed().lte(speed);
  }

  /**
   * Get the {@link SmartMotorController} Mechanism velocity representing the FlyWheel.
   *
   * @return FlyWheel {@link AngularVelocity}
   */
  public AngularVelocity getSpeed() {
    return m_smc.getMechanismVelocity();
  }

  /**
   * Get the {@link LinearVelocity} of the FlyWheel.
   *
   * @return FlyWheel {@link LinearVelocity}
   */
  public LinearVelocity getLinearVelocity() {
    return m_config.getLinearVelocity(m_smc.getMechanismVelocity());
  }

  /**
   * FlyWheel is near a speed.
   *
   * @param speed  {@link AngularVelocity} to be near.
   * @param within {@link AngularVelocity} within.
   * @return True if the FlyWheel is near another speed.
   */
  public boolean isNear(AngularVelocity speed, AngularVelocity within) {
    return getSpeed().isNear(speed, within);
  }

  /**
   * Mechanism velocity setpoint.
   *
   * @return {@link AngularVelocity} setpoint of the FlyWheel.
   */
  public Optional<AngularVelocity> getMechanismSetpointVelocity() {
    return m_smc.getMechanismSetpointVelocity();
  }

  /**
   * Set the FlyWheel to the given speed.
   *
   * @param speed {@link LinearVelocity} to go to.
   */
  @Override
  public void setMeasurementVelocitySetpoint(LinearVelocity speed) {
    m_smc.startClosedLoopController();
    m_smc.setVelocity(m_config.getAngularVelocity(speed));
  }

  @Override
  public void simIterate() {
    if (m_dcmotorSim.isPresent() && m_smc.getSimSupplier().isPresent()) {
      m_smc.getSimSupplier().get().updateSimState();
      m_smc.simIterate();
      m_smc.getSimSupplier().get().starveUpdateSim();

      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_dcmotorSim.get().getCurrentDraw()));
      visualizationUpdate();
    }
  }

  @Override
  public void updateTelemetry() {
    //    m_telemetry.updatePosition(getAngle());
    //    m_motor.getMechanismPositionSetpoint().ifPresent(m_setpoint ->
    //    m_telemetry.updateSetpoint(m_setpoint));
    m_smc.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  /**
   * Updates the angle of the mechanism ligament to match the current angle of the FlyWheel.
   */
  @Override
  public void visualizationUpdate() {
    if (m_config.isUsingSpeedometerSimulation() && m_config.getSpeedometerMaxVelocity().isPresent()) {
      mechanismLigament.setAngle(270 - m_smc.getMechanismVelocity().in(RPM) / m_config.getSpeedometerMaxVelocity().get().in(RPM) * 180);
    } else {
      mechanismLigament.setAngle(m_smc.getMechanismPosition().in(Degrees));
    }
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined
   * in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition() {
    Translation3d mechanismTranslation = new Translation3d(mechanismLigament.getLength(), new Rotation3d(0, 0, mechanismLigament.getAngle()));
    if (m_config.getMechanismPositionConfig().getRelativePosition().isPresent()) {
      return m_config.getMechanismPositionConfig().getRelativePosition().get().plus(mechanismTranslation);
    }
    return mechanismTranslation;
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("FlyWheel");
  }

  /**
   * Get the {@link FlyWheelConfig} object for this {@link FlyWheel}
   *
   * @return The {@link FlyWheelConfig} object for this {@link FlyWheel}
   */
  public FlyWheelConfig getShooterConfig() {
    return m_config;
  }
}
