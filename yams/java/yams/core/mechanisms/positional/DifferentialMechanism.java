// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.NoSuchElementException;
import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Color;
import org.wpilib.util.Color8Bit;
import yams.core.exceptions.DifferentialMechanismConfigurationException;
import yams.core.mechanisms.config.DifferentialMechanismConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.simulation.DCMotorSimSupplier;

/**
 * A differential mechanism for FRC robots driven by two coordinated motors whose combined motion
 * produces two distinct and independent outputs.
 *
 * <p>In a differential drive mechanism the two motors do <b>not</b> independently move two
 * separate joints. Instead, their outputs are mixed mathematically:</p>
 * <ul>
 * <li><b>Tilt</b>: controlled by the <em>sum</em> of the left and right motor positions
 * ({@code (left + right) / 2}). When both motors turn in the same direction the mechanism
 * tilts up or down.</li>
 * <li><b>Twist</b>: controlled by the <em>difference</em> of the left and right motor
 * positions ({@code (left - right) / 2}). When the motors turn in opposite directions the
 * mechanism rotates about its longitudinal axis.</li>
 * </ul>
 *
 * <p>A common FRC use-case is a differential wrist at the end of an arm where one motor controls
 * pitch while the other controls roll, allowing the intake or end-effector to be positioned in
 * two degrees of freedom with fewer motors than a traditional two-joint wrist.</p>
 *
 * <p>This core class holds mechanism state, physics simulation, and measurement logic only.
 * Command factories live on {@link yams.commands2.mechanisms.DifferentialMechanism}, which
 * extends this class and has a full usage example in its Javadoc.
 */
public class DifferentialMechanism extends SmartPositionalMechanism {
  /**
   * Left {@link SmartMotorController}
   */
  private final SmartMotorController        m_leftSMC;
  /**
   * Right {@link SmartMotorController}
   */
  private final SmartMotorController        m_rightSMC;
  /**
   * Differential Mechanism config.
   */
  private final DifferentialMechanismConfig m_config;
  /**
   * Simulation for the left motor.
   */
  private Optional<DCMotorSim>              m_leftSim  = Optional.empty();
  /**
   * Simulation for the right motor.
   */
  private Optional<DCMotorSim>              m_rightSim = Optional.empty();
  /**
   * Twist Ligament
   */
  private MechanismLigament2d               m_twistLigament;
  /**
   * Twist root
   */
  private MechanismRoot2d                   m_twistRoot;
  /**
   * Arm length.
   */
  private final Distance                    m_armLength;
  /**
   * Tilt root.
   */
  private Translation2d                     m_tiltRoot;

  /**
   * Constructor for the Differential mechanism.
   *
   * @param diffConfig Lower {@link DifferentialMechanismConfig} to use.
   * @implNote Protected so only {@link yams.commands2.mechanisms.DifferentialMechanism} can
   *           construct this.
   * @throws NoSuchElementException                      if the left or right
   *                                                     {@link SmartMotorController} was not set on
   *                                                     the {@link DifferentialMechanismConfig}.
   * @throws DifferentialMechanismConfigurationException if the starting tilt or twist angle is not
   *                                                     set.
   * @throws DifferentialMechanismConfigurationException if the length is not set.
   * @throws DifferentialMechanismConfigurationException if running in simulation and the MOI is not
   *                                                     set.
   */
  protected DifferentialMechanism(DifferentialMechanismConfig diffConfig) {
    m_config = diffConfig;
    m_leftSMC = diffConfig.getLeftMotorController();
    m_rightSMC = diffConfig.getRightMotorController();

    // Check that the starting angle is defined
    if (m_config.getStartingTiltAngle().isEmpty() || m_config.getStartingTwistAngle().isEmpty()) {
      throw new DifferentialMechanismConfigurationException("Starting angle is empty", "Cannot create simulation.", "withTwistStartingPosition(Angle) OR " + "DifferentialMechanismConfig.withTiltStartingPosition(Angle)");
    }

    // Check that the arm lengths are defined
    if (m_config.getLength().isEmpty()) {
      throw new DifferentialMechanismConfigurationException("Lengths must be defined to calculate current end position of the DifferentialMechanism!", "Cannot create mechanism", "withLength(Distance)");
    }
    m_armLength = m_config.getLength().get();

    // Setup root mechanism position for calculations.
    var mechPosCfg = m_config.getMechanismPositionConfig();

    // Seed the relative encoder
    m_leftSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_leftSMC.seedRelativeEncoder();
    });
    m_rightSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_rightSMC.seedRelativeEncoder();
    });

    // Setup telemetry
    m_config.getTelemetryName().ifPresent(name -> {
      m_telemetry.setupTelemetry(getName());
      m_telemetry.addMotorController("left", m_leftSMC);
      m_telemetry.addMotorController("right", m_rightSMC);
    });

    if (RobotBase.isSimulation()) {
      var startingtilt = m_config.getStartingTiltAngle().get();
      var startingtwist = m_config.getStartingTwistAngle().get();
      var startingleft = m_config.getLeftMechanismPosition(startingtilt, startingtwist);
      var startingright = m_config.getRightMechanismPosition(startingtilt, startingtwist);
      m_tiltRoot = new Translation2d(m_armLength.in(Meters), 0);

      // Setup Sim
      m_leftSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(m_leftSMC.getDCMotor(), m_config.getMOI(), m_leftSMC.getConfig().getGearing().getMechanismToRotorRatio()), m_leftSMC.getDCMotor()));
      m_rightSim = Optional.of(new DCMotorSim(Models.singleJointedArmFromPhysicalConstants(m_rightSMC.getDCMotor(), m_config.getMOI(), m_rightSMC.getConfig().getGearing().getMechanismToRotorRatio()), m_rightSMC.getDCMotor()));
      m_leftSMC.setSimSupplier(new DCMotorSimSupplier(m_leftSim.get(), m_leftSMC));
      m_rightSMC.setSimSupplier(new DCMotorSimSupplier(m_rightSim.get(), m_rightSMC));

      m_mechanismWindow = new Mechanism2d(mechPosCfg.getWindowXDimension(m_armLength).plus(Inches.of(4)).in(Meters), mechPosCfg.getWindowYDimension(m_armLength.plus(Inches.of(4))).in(Meters));
      m_mechanismRoot = m_mechanismWindow.getRoot("Root", m_tiltRoot.getX(), m_tiltRoot.getY());
      m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(" tilt", m_armLength.in(Meters), startingtilt.in(Degrees), 7, m_config.getSimColor()));
      m_twistRoot = m_mechanismWindow.getRoot("Twist Root", m_armLength.in(Meters), m_armLength.in(Meters));
      m_twistLigament = m_twistRoot.append(new MechanismLigament2d(" twist", Inches.of(4).in(Meters), startingtwist.in(Degrees), 6, new Color8Bit(Color.RED)));

      publishMechanismWindow();

      m_leftSMC.setupSimulation();
      m_rightSMC.setupSimulation();

      m_leftSim.get().setAngle(startingleft.in(Radians));
      m_rightSim.get().setAngle(startingright.in(Radians));
    }

    // Apply configs
    m_config.applyConfig();
  }

  /**
   * Get the left {@link SmartMotorController} of the mechanism.
   *
   * @return Left {@link SmartMotorController}.
   */
  public SmartMotorController getLeftMotorController() {
    return m_leftSMC;
  }

  /**
   * Get the right {@link SmartMotorController} of the mechanism.
   *
   * @return Right {@link SmartMotorController}.
   */
  public SmartMotorController getRightMotorController() {
    return m_rightSMC;
  }

  /**
   * Get the twist {@link Angle} of the mechanism.
   *
   * @return Twist {@link Angle}.
   */
  public Angle getTwistPosition() {
    if (m_config.getTwistAngleSupplier().isPresent()) {
      return m_config.getTwistAngleSupplier().get().get();
    }
    return m_config.getTwistAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
  }

  /**
   * Get the tilt {@link Angle} of the mechanism.
   *
   * @return Tilt {@link Angle}.
   */
  public Angle getTiltPosition() {
    if (m_config.getTiltAngleSupplier().isPresent()) {
      return m_config.getTiltAngleSupplier().get().get();
    }
    return m_config.getTiltAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
  }

  /**
   * Get the {@link DifferentialMechanismConfig} for this mechanism.
   *
   * @return {@link DifferentialMechanismConfig} for this mechanism.
   */
  public DifferentialMechanismConfig getDifferentialMechanismConfig() {
    return m_config;
  }

  @Override
  public void updateTelemetry() {
    m_leftSMC.updateTelemetry();
    m_rightSMC.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  @Override
  public void simIterate() {
    if (m_leftSim.isPresent() && m_leftSMC.getSimSupplier().isPresent() && m_rightSim.isPresent() && m_rightSMC.getSimSupplier().isPresent()) {
      m_leftSMC.getSimSupplier().get().updateSimState();
      m_leftSMC.simIterate();
      m_leftSMC.getSimSupplier().get().starveUpdateSim();
      m_rightSMC.getSimSupplier().get().updateSimState();
      m_rightSMC.simIterate();
      m_rightSMC.getSimSupplier().get().starveUpdateSim();
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_leftSim.get().getCurrentDraw(), m_rightSim.get().getCurrentDraw()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the mechanism ligament with the current angle of the Differential Mechanism.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
   */
  @Override
  public void visualizationUpdate() {
    var twistAngle = getTwistPosition();
    var tiltAngle = getTiltPosition();
    var twistRoot = new Translation2d(m_armLength.in(Meters), Rotation2d.fromDegrees(tiltAngle.in(Degrees))).plus(m_tiltRoot);
    m_mechanismLigament.setAngle(tiltAngle.in(Degrees));

    m_twistRoot.setPosition(twistRoot.getX(), twistRoot.getY());
    m_twistLigament.setAngle(twistAngle.in(Degrees));
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
    return new Translation3d(m_armLength.in(Meters), new Rotation3d(Degrees.of(0), getTiltPosition(), Degrees.of(0)));
  }

  @Override
  public String getName() {
    return m_config.getTelemetryName().orElse("DifferentialMechanism");
  }

  /**
   * Not supported for {@link DifferentialMechanism}.
   *
   * @return Never returns.
   * @throws RuntimeException always, since max limits are not supported for this mechanism.
   */
  @Override
  public boolean isAtMax() {
    throw new RuntimeException("Unsupported operation");
  }

  /**
   * Not supported for {@link DifferentialMechanism}.
   *
   * @return Never returns.
   * @throws RuntimeException always, since min limits are not supported for this mechanism.
   */
  @Override
  public boolean isAtMin() {
    throw new RuntimeException("Unsupported operation");
  }
}
