// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.simulation.BatterySim;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Pair;
import yams.core.exceptions.DoubleJointedArmConfigurationException;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.ArmSimSupplier;

/**
 * A two-segment arm where each joint has its own independent motor controller.
 *
 * <p>Unlike a differential mechanism, the joints in a {@code DoubleJointedArm} are driven
 * individually: the lower (shoulder) motor directly controls the first segment and the upper
 * (elbow) motor directly controls the second segment. Kinematics are applied in software to
 * convert between Cartesian end-effector positions and the required joint angles.</p>
 *
 * <p>This mechanism is well-suited to FRC scoring subsystems that require both high reach and
 * precise end-effector placement. A long lower segment provides reach, while the upper segment
 * allows the end-effector to be folded back under frame perimeter when traveling or extended
 * far over field elements when scoring.</p>
 *
 * <h2>Construction Example</h2>
 * <pre>{@code
 * // Each ArmConfig must specify the motor, gearing, arm length, hard limits,
 * // and starting angle before the DoubleJointedArm can be constructed.
 * ArmConfig lowerConfig = new ArmConfig(lowerSMC)
 *     .withLength(Inches.of(24))
 *     .withHardLimits(Degrees.of(0), Degrees.of(120))
 *     .withStartingPosition(Degrees.of(0))
 *     .withTelemetry("Shoulder", TelemetryVerbosity.HIGH);
 *
 * ArmConfig upperConfig = new ArmConfig(upperSMC)
 *     .withLength(Inches.of(20))
 *     .withHardLimits(Degrees.of(0), Degrees.of(120))
 *     .withStartingPosition(Degrees.of(90))
 *     .withTelemetry("Elbow", TelemetryVerbosity.HIGH);
 *
 * DoubleJointedArm arm = new DoubleJointedArm(lowerConfig, upperConfig);
 * }</pre>
 *
 * <p>This core class holds arm state, kinematics, and physics simulation only. Command and
 * Trigger factories live on {@link yams.commands2.mechanisms.DoubleJointedArm}, which extends
 * this class and has control examples in its Javadoc.
 */
public class DoubleJointedArm extends SmartPositionalMechanism {
  /**
   * Upper arm {@link SmartMotorController}
   */
  private final SmartMotorController    m_upperSMC;
  /**
   * Lower arm {@link SmartMotorController}
   */
  private final SmartMotorController    m_lowerSMC;
  /**
   * Arm config.
   */
  private final ArmConfig               m_lowerArmConfig;
  /**
   * Arm config.
   */
  private final ArmConfig               m_upperArmConfig;
  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim> m_lowerArmSim = Optional.empty();
  /**
   * Simulation for the arm.
   */
  private Optional<SingleJointedArmSim> m_upperArmSim = Optional.empty();
  /**
   * Lower ligament
   */
  private MechanismLigament2d           m_lowerLigament;
  /**
   * Upper root
   */
  private MechanismRoot2d               m_upperRoot;
  /**
   * Upper ligament.
   */
  private MechanismLigament2d           m_upperLigament;
  /**
   * Lower Arm root position in meters.
   */
  private final Translation2d           m_lowerArmRootPos;
  /**
   * Lower arm length used for trig calculations on current position.
   */
  private final Distance                m_lowerArmLength;
  /**
   * Upper arm length used for trig calculations on current position.
   */
  private final Distance                m_upperArmLength;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param lowerConfig Lower {@link ArmConfig} to use.
   * @param lowerSMC    {@link SmartMotorController} driving the lower joint.
   * @param upperConfig Upper {@link ArmConfig} to use.
   * @param upperSMC    {@link SmartMotorController} driving the upper joint.
   * @implNote Protected so only {@link yams.commands2.mechanisms.DoubleJointedArm} can construct
   *           this.
   * @throws DoubleJointedArmConfigurationException if either {@link SmartMotorControllerConfig}
   *                                                has no starting position, either {@link ArmConfig}
   *                                                has no length, or running in simulation and either
   *                                                {@link ArmConfig} is missing its lower or upper hard
   *                                                limit.
   */
  protected DoubleJointedArm(ArmConfig lowerConfig, SmartMotorController lowerSMC, ArmConfig upperConfig, SmartMotorController upperSMC) {
    m_lowerArmConfig = lowerConfig;
    m_upperArmConfig = upperConfig;
    m_lowerSMC = lowerSMC;
    m_upperSMC = upperSMC;
    SmartMotorControllerConfig<?> lowerSMCConfig = lowerSMC.getConfig();
    SmartMotorControllerConfig<?> upperSMCConfig = upperSMC.getConfig();

    // Check that the starting angle is defined
    if (lowerSMCConfig.getStartingPosition().isEmpty() || upperSMCConfig.getStartingPosition().isEmpty()) {
      throw new DoubleJointedArmConfigurationException("Arm starting angle is empty", "Cannot create simulation.", "SmartMotorControllerConfig.withStartingPosition(Angle)");
    }

    // Check that the arm lengths are defined
    if (lowerConfig.getLength().isEmpty() || upperConfig.getLength().isEmpty()) {
      throw new DoubleJointedArmConfigurationException("Arm lengths must be defined to calculate current end position of the Double Jointed " + "Arm!", "Cannot create mechanism", "withLength(Distance)");
    }
    m_lowerArmLength = lowerConfig.getLength().get();
    m_upperArmLength = upperConfig.getLength().get();

    // Setup root mechanism position for calculations.
    var lowerMechPosCfg = lowerConfig.getMechanismPositionConfig();
    var upperMechPosCfg = upperConfig.getMechanismPositionConfig();
    m_lowerArmRootPos = new Translation2d(m_lowerArmLength.plus(m_upperArmLength).in(Meters), 0);

    // Seed the relative encoder
    m_lowerSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_lowerSMC.seedRelativeEncoder();
    });
    m_upperSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_upperSMC.seedRelativeEncoder();
    });

    // Setup telemetry
    if (lowerConfig.getTelemetryName().isPresent() || upperConfig.getTelemetryName().isPresent()) {
      m_telemetry.setupTelemetry(getName());
    }
    lowerConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.addMotorController("lower", m_lowerSMC);
    });
    upperConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.addMotorController("upper", m_upperSMC);
    });

    if (RobotBase.isSimulation()) {
      if (lowerConfig.getLowerHardLimit().isEmpty() || upperConfig.getLowerHardLimit().isEmpty()) {
        throw new DoubleJointedArmConfigurationException("Arm lower hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }
      if (lowerConfig.getUpperHardLimit().isEmpty() || upperConfig.getUpperHardLimit().isEmpty()) {
        throw new DoubleJointedArmConfigurationException("Arm upper hard limit is empty", "Cannot create simulation.", "withHardLimits(Angle,Angle)");
      }

      // Setup Sim
      m_lowerArmSim = Optional.of(new SingleJointedArmSim(m_lowerSMC.getDCMotor(), m_lowerSMC.getConfig().getGearing().getMechanismToRotorRatio(), lowerSMCConfig.getMOI(), lowerConfig.getLength().get().in(Meters), lowerConfig.getLowerHardLimit()
          .get().in(Radians), lowerConfig.getUpperHardLimit().get().in(Radians), true, lowerSMCConfig.getStartingPosition().get().in(Radians), 0.002 / 4096.0, 0.0)); // Add noise with a std-dev of 1 tick
      m_lowerSMC.setSimSupplier(new ArmSimSupplier(m_lowerArmSim.get(), m_lowerSMC));
      m_upperArmSim = Optional.of(new SingleJointedArmSim(m_upperSMC.getDCMotor(), m_upperSMC.getConfig().getGearing().getMechanismToRotorRatio(), upperSMCConfig.getMOI(), m_upperArmLength.in(Meters), upperConfig.getLowerHardLimit().get().in(
          Radians), upperConfig.getUpperHardLimit().get().in(Radians), true, upperSMCConfig.getStartingPosition().get().in(Radians), 0.002 / 4096.0, 0.0)); // Add noise with a std-dev of 1 tick
      m_upperSMC.setSimSupplier(new ArmSimSupplier(m_upperArmSim.get(), m_upperSMC));

      var lowerStartingAngle = lowerSMCConfig.getStartingPosition().get();
      var upperStartingAngle = upperSMCConfig.getStartingPosition().get();

      var upperArmRootPos = getJoint(m_lowerArmLength, lowerStartingAngle, m_lowerArmRootPos);

      var windowX = lowerMechPosCfg.getWindowXDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowXDimension(m_upperArmLength).in(Meters);
      var windowY = lowerMechPosCfg.getWindowYDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowYDimension(m_upperArmLength).in(Meters);

      m_mechanismWindow = new Mechanism2d(windowX, windowY);
      m_mechanismRoot = m_mechanismWindow.getRoot("Lower Root", m_lowerArmRootPos.getX(), m_lowerArmRootPos.getY());
      m_lowerLigament = m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(" lower", m_lowerArmLength.in(Meters), lowerStartingAngle.in(Degrees), 7, lowerConfig.getSimColor()));
      m_upperRoot = m_mechanismWindow.getRoot("Upper Root", upperArmRootPos.getX(), upperArmRootPos.getY());
      m_upperLigament = m_upperRoot.append(new MechanismLigament2d("upper", m_upperArmLength.in(Meters), upperStartingAngle.in(Degrees), 6, upperConfig.getSimColor()));
      publishMechanismWindow();

      m_upperSMC.setupSimulation();
      m_lowerSMC.setupSimulation();
    }
  }

  /**
   * Get the joint {@link Translation2d} of the arm in Meters.
   *
   * @param armLen   {@link Distance} length of the arm.
   * @param armAngle {@link Angle} angle of the arm.
   * @param offset   {@link Translation2d} root position to offset by.
   * @return {@link Translation2d} of the joint.
   */
  private Translation2d getJoint(Distance armLen, Angle armAngle, Translation2d offset) {
    return new Translation2d(armLen.times(Math.cos(armAngle.in(Radians))).in(Meters), armLen.times(Math.sin(armAngle.in(Radians))).in(Meters)).plus(offset);
  }

  /**
   * Get the {@link Translation2d} of the double jointed arm.
   *
   * @return {@link Translation2d} of the double jointed arm.
   */
  public Translation2d getPosition() {
    return getJoint(m_upperArmLength, m_upperSMC.getMechanismPosition(), getJoint(m_lowerArmLength, m_lowerSMC.getMechanismPosition(), Translation2d.ZERO));
  }

  /**
   * Inverse Kinematics for a DoubleJointedArm
   *
   * @param translation Translations from root in Meters.
   * @param invert      Invert the elbow.
   * @return {@link Pair} with the shoulder angle then elbow angle.
   */
  public Pair<Angle, Angle> getAnglesForPosition(Translation2d translation, boolean invert) {
    var x = Meters.of(translation.getX()).in(Meters);
    var y = Meters.of(translation.getY()).in(Meters);
    var l1 = m_lowerArmLength.in(Meters);
    var l2 = m_upperArmLength.in(Meters);
    var theta2 = Math.acos(((x * x) + (y * y) - ((l1 * l1) + (l2 * l2))) / (2 * l1 * l2));
    theta2 = invert ? -theta2 : theta2;

    var theta1 = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(theta2), l1 + (l2 * Math.cos(theta2)));
    if (invert) {
      theta2 += Math.PI / 2;
    }
    return Pair.of(Radians.of(theta1), Radians.of(theta2));
  }

  /**
   * Get the lower (shoulder) {@link SmartMotorController}.
   *
   * @return Lower {@link SmartMotorController}.
   */
  public SmartMotorController getLowerMotorController() {
    return m_lowerSMC;
  }

  /**
   * Get the upper (elbow) {@link SmartMotorController}.
   *
   * @return Upper {@link SmartMotorController}.
   */
  public SmartMotorController getUpperMotorController() {
    return m_upperSMC;
  }

  /**
   * Get the lower (shoulder) {@link ArmConfig}.
   *
   * @return Lower {@link ArmConfig}.
   */
  public ArmConfig getLowerArmConfig() {
    return m_lowerArmConfig;
  }

  /**
   * Get the upper (elbow) {@link ArmConfig}.
   *
   * @return Upper {@link ArmConfig}.
   */
  public ArmConfig getUpperArmConfig() {
    return m_upperArmConfig;
  }

  /**
   * Is near the target within tolerance
   *
   * @param target    Target to check
   * @param tolerance Tolerance
   * @return Boolean
   */
  public boolean isNear(Translation2d target, Distance tolerance) {
    return getPosition().getDistance(target) < tolerance.in(Meters);
  }

  //    def inv_kinematics(self, pos, invert = False):
  //        """Inverse kinematics for a target position pos (x,y). Invert controls elbow
  //        direction.""" [x,y] = pos.flat
  //        theta2 = np.arccos((x*x + y*y - (self.l1*self.l1 + self.l2*self.l2)) / \
  //            (2*self.l1*self.l2))
  //
  //        if invert:
  //            theta2 = -theta2
  //
  //        theta1 = np.arctan2(y, x) - np.arctan2(self.l2*np.sin(theta2), self.l1 +
  //        self.l2*np.cos(theta2)) return np.array([[theta1, theta2]]).T

  @Override
  public void updateTelemetry() {
    m_lowerSMC.updateTelemetry();
    m_upperSMC.updateTelemetry();
    m_telemetry.updateLoopTime();
  }

  @Override
  public void simIterate() {
    if (m_lowerArmSim.isPresent() && m_lowerSMC.getSimSupplier().isPresent() && m_upperArmSim.isPresent() && m_upperSMC.getSimSupplier().isPresent()) {
      m_lowerSMC.getSimSupplier().get().updateSimState();
      m_lowerSMC.simIterate();
      m_lowerSMC.getSimSupplier().get().starveUpdateSim();
      m_upperSMC.getSimSupplier().get().updateSimState();
      m_upperSMC.simIterate();
      m_upperSMC.getSimSupplier().get().starveUpdateSim();
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_lowerArmSim.get().getCurrentDraw(), m_upperArmSim.get().getCurrentDraw()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the mechanism ligament with the current angle of the arm.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
   */
  @Override
  public void visualizationUpdate() {
    var lowerArmAngle = getLowerAngle();
    var upperArmAngle = getUpperAngle();
    var jointPos = getJoint(m_lowerArmLength, lowerArmAngle, m_lowerArmRootPos);
    m_lowerLigament.setAngle(lowerArmAngle.in(Degrees));
    m_upperLigament.setAngle(upperArmAngle.in(Degrees));
    m_upperRoot.setPosition(jointPos.getX(), jointPos.getY());
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
    var pos = getPosition();
    var x = pos.getX();
    var z = pos.getY();
    return new Translation3d(x, 0, z);
  }

  @Override
  public String getName() {
    return "DoubleJointedArm_" + m_lowerArmConfig.getTelemetryName().orElse("Lower") + "_" + m_upperArmConfig.getTelemetryName().orElse("Upper");
  }

  /**
   * Get the shoulder angle of the DoubleJointedArm.
   *
   * @return {@link Angle} of the shoulder.
   */
  public Angle getLowerAngle() {
    return m_lowerSMC.getMechanismPosition();
  }

  /**
   * Get the elbow angle of the DoubleJointedArm
   *
   * @return {@link Angle} of the elbow.
   */
  public Angle getUpperAngle() {
    return m_upperSMC.getMechanismPosition();
  }

  /**
   * Not supported for {@link DoubleJointedArm}.
   *
   * @return Never returns.
   * @throws RuntimeException always, since max limits are not supported for this mechanism.
   */
  @Override
  public boolean isAtMax() {
    throw new RuntimeException("Unsupported operation");
  }

  /**
   * Not supported for {@link DoubleJointedArm}.
   *
   * @return Never returns.
   * @throws RuntimeException always, since min limits are not supported for this mechanism.
   */
  @Override
  public boolean isAtMin() {
    throw new RuntimeException("Unsupported operation");
  }
}
