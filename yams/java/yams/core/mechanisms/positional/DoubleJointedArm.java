// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.mechanisms.positional;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.Seconds;

import java.util.Optional;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.smartdashboard.Mechanism2d;
import org.wpilib.smartdashboard.MechanismLigament2d;
import org.wpilib.smartdashboard.MechanismRoot2d;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Distance;
import org.wpilib.util.Pair;
import yams.core.exceptions.DoubleJointedArmConfigurationException;
import yams.core.math.DoubleJointedArmDynamics;
import yams.core.math.DoubleJointedArmPath;
import yams.core.math.DoubleJointedArmPathPlanner;
import yams.core.mechanisms.config.ArmConfig;
import yams.core.mechanisms.config.DoubleJointedArmConfig;
import yams.core.mechanisms.config.MechanismPositionConfig.Plane;
import yams.core.mechanisms.config.MechanismPositionConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.SmartMotorControllerConfig;
import yams.core.motorcontrollers.simulation.DoubleJointedArmSim;
import yams.core.motorcontrollers.simulation.DoubleJointedArmSimSupplier;
import yams.core.motorcontrollers.simulation.DoubleJointedArmSimSupplier.Joint;
import yams.core.telemetry.DoubleJointedArmTelemetry;

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
 * <p>Angles are the motor controllers' joint angles: the lower arm's from horizontal, and the upper arm's from
 * horizontal by default, as on an arm whose elbow is driven from the base, e.g. by a chain through the shoulder, or from
 * the lower arm, as on an arm with its elbow motor on the lower arm; see
 * {@link DoubleJointedArmConfig#withUpperAngleReference(DoubleJointedArmConfig.UpperAngleReference)}. Positions are in
 * the arm's frame, from the shoulder with X away from it and Y up.
 *
 * <h2>Gravity</h2>
 * Each joint's {@link org.wpilib.math.controller.ArmFeedforward} only knows its own angle, but gravity on the shoulder
 * depends on where the upper arm is too. Moves to an angle or position change each joint's kG as the arm moves, so it
 * holds the arm; see {@link #updateKg()}.
 *
 * <h2>Exclusion zones</h2>
 * Boxes the arm must never enter, such as the robot's frame and bumpers, are set on a {@link DoubleJointedArmConfig}.
 * Moves to an angle or position then follow a path planned with A* in joint space that keeps both segments out of
 * every zone the whole way; see {@link DoubleJointedArmPathFollower}. Duty cycle control doesn't follow a path.
 *
 * <h2>Simulation</h2>
 * Both joints are simulated together by a {@link DoubleJointedArmSim}, so the shoulder carries the upper arm's weight
 * and inertia, and swinging either arm pushes on the other, for either upper angle reference.
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
   * Double jointed arm config.
   */
  private final DoubleJointedArmConfig  m_config;
  /**
   * Simulation of both joints together.
   */
  private Optional<DoubleJointedArmSim> m_armSim      = Optional.empty();
  /**
   * Plans paths around the exclusion zones.
   */
  private final DoubleJointedArmPathPlanner m_planner;
  /**
   * Equations of motion, for the simulation and kG compensation.
   */
  private final DoubleJointedArmDynamics m_dynamics;
  /**
   * How close each joint must get to a waypoint before the arm moves on to the next.
   */
  private Angle                          m_pathTolerance;
  /**
   * Each joint's kG with the arm held straight out horizontally, NaN until known.
   */
  private final double[]                 m_kgStraight   = {Double.NaN, Double.NaN};
  /**
   * Each joint's kG as last sent to its motor controller, NaN before it is sent.
   */
  private final double[]                 m_kgSent       = {Double.NaN, Double.NaN};
  /**
   * Follows paths for every command that moves the arm to an angle or position. Only one command drives the arm at a
   * time, so they share it, each giving it a new goal as it starts.
   */
  private final DoubleJointedArmPathFollower     m_pathFollower       = new DoubleJointedArmPathFollower(this);
  /**
   * The arm's own telemetry.
   */
  private Optional<DoubleJointedArmTelemetry>    m_armTelemetry       = Optional.empty();
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
   * Constructor for the Arm mechanism, without exclusion zones.
   *
   * @param lowerConfig Lower {@link ArmConfig} to use.
   * @param lowerSMC    {@link SmartMotorController} driving the lower joint.
   * @param upperConfig Upper {@link ArmConfig} to use.
   * @param upperSMC    {@link SmartMotorController} driving the upper joint.
   * @implNote Protected so only {@link yams.commands2.mechanisms.DoubleJointedArm} can construct
   *           this.
   * @throws DoubleJointedArmConfigurationException if either {@link SmartMotorControllerConfig}
   *                                                has no starting position.
   * @throws DoubleJointedArmConfigurationException if either {@link ArmConfig} has no length.
   * @throws DoubleJointedArmConfigurationException if running in simulation and either
   *                                                {@link ArmConfig} is missing its lower or upper
   *                                                hard limit.
   */
  protected DoubleJointedArm(ArmConfig lowerConfig, SmartMotorController lowerSMC, ArmConfig upperConfig, SmartMotorController upperSMC) {
    this(new DoubleJointedArmConfig(lowerConfig, lowerSMC, upperConfig, upperSMC));
  }

  /**
   * Constructor for the Arm mechanism.
   *
   * @param config {@link DoubleJointedArmConfig} with both arms and the exclusion zones.
   * @implNote Protected so only {@link yams.commands2.mechanisms.DoubleJointedArm} can construct
   *           this.
   * @throws DoubleJointedArmConfigurationException if either {@link SmartMotorControllerConfig}
   *                                                has no starting position.
   * @throws DoubleJointedArmConfigurationException if either {@link ArmConfig} has no length.
   * @throws DoubleJointedArmConfigurationException if running in simulation and either
   *                                                {@link ArmConfig} is missing its lower or upper
   *                                                hard limit.
   */
  protected DoubleJointedArm(DoubleJointedArmConfig config) {
    m_config = config;
    ArmConfig            lowerConfig = config.getLowerConfig();
    SmartMotorController lowerSMC    = config.getLowerMotorController();
    ArmConfig            upperConfig = config.getUpperConfig();
    SmartMotorController upperSMC    = config.getUpperMotorController();
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
    m_dynamics = new DoubleJointedArmDynamics(m_lowerArmLength, lowerSMCConfig.getMOI(), m_upperArmLength, upperSMCConfig.getMOI(), config.getUpperAngleReference());
    m_pathTolerance = config.getPathTolerance();

    // Setup root mechanism position for calculations.
    var lowerMechPosCfg = lowerConfig.getMechanismPositionConfig();
    var upperMechPosCfg = upperConfig.getMechanismPositionConfig();
    m_lowerArmRootPos = new Translation2d(m_lowerArmLength.plus(m_upperArmLength).in(Meters), 0);

    // The planner keeps each joint within its hard limits, or else its motor controller's mechanism limits.
    m_planner = new DoubleJointedArmPathPlanner(m_lowerArmLength, m_upperArmLength,
                                                lowerConfig.getLowerHardLimit().or(lowerSMCConfig::getMechanismLowerLimit).orElse(Radians.of(Double.NEGATIVE_INFINITY)),
                                                lowerConfig.getUpperHardLimit().or(lowerSMCConfig::getMechanismUpperLimit).orElse(Radians.of(Double.POSITIVE_INFINITY)),
                                                upperConfig.getLowerHardLimit().or(upperSMCConfig::getMechanismLowerLimit).orElse(Radians.of(Double.NEGATIVE_INFINITY)),
                                                upperConfig.getUpperHardLimit().or(upperSMCConfig::getMechanismUpperLimit).orElse(Radians.of(Double.POSITIVE_INFINITY)),
                                                config.getExclusionZones(), config.getExclusionZoneClearance(), config.getPathResolution(),
                                                config.getPathTolerance(), config.getUpperAngleReference());

    // Seed the relative encoder
    m_lowerSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_lowerSMC.seedRelativeEncoder();
    });
    m_upperSMC.getConfig().getExternalEncoder().ifPresent(encoder -> {
      m_upperSMC.seedRelativeEncoder();
    });

    // Setup telemetry
    if (lowerConfig.getTelemetryName().isPresent() || upperConfig.getTelemetryName().isPresent() || config.getTelemetryName().isPresent()) {
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

      // Setup Sim: both joints together, as each moves the other.
      m_armSim = Optional.of(new DoubleJointedArmSim(m_dynamics, m_lowerSMC.getDCMotor(), lowerSMCConfig.getGearing().getMechanismToRotorRatio(), lowerConfig.getLowerHardLimit().get(),
                                                     lowerConfig.getUpperHardLimit().get(), lowerSMCConfig.getStartingPosition().get(), m_upperSMC.getDCMotor(),
                                                     upperSMCConfig.getGearing().getMechanismToRotorRatio(), upperConfig.getLowerHardLimit().get(), upperConfig.getUpperHardLimit().get(),
                                                     upperSMCConfig.getStartingPosition().get(), true));
      m_lowerSMC.setSimSupplier(new DoubleJointedArmSimSupplier(m_armSim.get(), Joint.LOWER, m_lowerSMC));
      m_upperSMC.setSimSupplier(new DoubleJointedArmSimSupplier(m_armSim.get(), Joint.UPPER, m_upperSMC));

      var lowerStartingAngle = lowerSMCConfig.getStartingPosition().get();
      var upperStartingAngle = Radians.of(m_dynamics.getUpperAngleFromHorizontal(lowerStartingAngle.in(Radians), upperSMCConfig.getStartingPosition().get().in(Radians)));

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

    config.getTelemetryVerbosity().ifPresent(verbosity -> {
      m_armTelemetry = Optional.of(new DoubleJointedArmTelemetry(verbosity, m_telemetry.getDataTable(), m_telemetry.getTuningTable(), this));
    });
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
   * Get where the end of the double jointed arm is, from the shoulder.
   *
   * @return {@link Translation2d} of the end of the arm, in meters.
   */
  public Translation2d getPosition() {
    return getJoint(m_upperArmLength, getUpperAngleFromHorizontal(), getElbowPosition());
  }

  /**
   * Get where the elbow is, from the shoulder.
   *
   * @return {@link Translation2d} of the elbow, in meters.
   */
  public Translation2d getElbowPosition() {
    return getJoint(m_lowerArmLength, getLowerAngle(), Translation2d.ZERO);
  }

  /**
   * Get the upper arm's angle from horizontal, whatever its joint angle is measured from.
   *
   * @return Upper arm {@link Angle} from horizontal.
   * @see DoubleJointedArmConfig#withUpperAngleReference(DoubleJointedArmConfig.UpperAngleReference)
   */
  public Angle getUpperAngleFromHorizontal() {
    return Radians.of(m_dynamics.getUpperAngleFromHorizontal(getLowerAngle().in(Radians), getUpperAngle().in(Radians)));
  }

  /**
   * Inverse Kinematics for a DoubleJointedArm: the joint angles that put the end of the arm at a position, as the
   * motor controllers measure them, the upper one from horizontal or from the lower arm. A position out of reach gives
   * the angles reaching straight toward it, as far as the arm goes.
   *
   * @param translation Position from the shoulder in meters, X away from it and Y up.
   * @param invert      Bend the elbow the other way.
   * @return {@link Pair} with the shoulder angle then elbow angle.
   */
  public Pair<Angle, Angle> getAnglesForPosition(Translation2d translation, boolean invert) {
    var x  = translation.getX();
    var y  = translation.getY();
    var l1 = m_lowerArmLength.in(Meters);
    var l2 = m_upperArmLength.in(Meters);
    // Angle of the upper arm from the lower arm, from the law of cosines.
    var bend = Math.acos(Math.max(-1, Math.min(1, ((x * x) + (y * y) - ((l1 * l1) + (l2 * l2))) / (2 * l1 * l2))));
    bend = invert ? -bend : bend;

    var lower = Math.atan2(y, x) - Math.atan2(l2 * Math.sin(bend), l1 + (l2 * Math.cos(bend)));
    return Pair.of(Radians.of(lower), Radians.of(m_dynamics.getUpperJointAngle(lower, lower + bend)));
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

  /**
   * Get the {@link DoubleJointedArmConfig}.
   *
   * @return {@link DoubleJointedArmConfig} the arm was made with.
   */
  public DoubleJointedArmConfig getDoubleJointedArmConfig() {
    return m_config;
  }

  /**
   * Whether exclusion zones are configured, so moves follow planned paths.
   *
   * @return True if there are exclusion zones.
   */
  public boolean hasExclusionZones() {
    return !m_config.getExclusionZones().isEmpty();
  }

  /**
   * Whether the arm at these angles is in an exclusion zone, or closer to one than the clearance.
   *
   * @param lowerAngle Lower arm angle, from horizontal.
   * @param upperAngle Upper arm angle, from horizontal.
   * @return True if either segment is in or too close to an exclusion zone.
   */
  public boolean isInExclusionZone(Angle lowerAngle, Angle upperAngle) {
    return m_planner.isInExclusionZone(lowerAngle, upperAngle);
  }

  /**
   * Whether the arm is in an exclusion zone, or closer to one than the clearance.
   *
   * @return True if either segment is in or too close to an exclusion zone.
   */
  public boolean isInExclusionZone() {
    return isInExclusionZone(getLowerAngle(), getUpperAngle());
  }

  /**
   * Plan a path from where the arm is to the given angles that never enters an exclusion zone. Without exclusion zones
   * the path is just the goal.
   *
   * @param lowerAngle Goal lower arm angle, from horizontal.
   * @param upperAngle Goal upper arm angle, from horizontal.
   * @return The {@link DoubleJointedArmPath}, or empty when the goal is outside the joint limits, in an exclusion zone,
   *     or can't be reached without entering one.
   */
  public Optional<DoubleJointedArmPath> planPath(Angle lowerAngle, Angle upperAngle) {
    if (!hasExclusionZones()) {
      return Optional.of(DoubleJointedArmPath.to(lowerAngle, upperAngle));
    }
    return m_planner.plan(getLowerAngle().in(Radians), getUpperAngle().in(Radians), lowerAngle.in(Radians), upperAngle.in(Radians));
  }

  /**
   * How close each joint must get to a waypoint before the arm moves on to the next.
   *
   * @return Path tolerance {@link Angle}.
   * @see DoubleJointedArmConfig#withPathTolerance(Angle)
   */
  public Angle getPathTolerance() {
    return m_pathTolerance;
  }

  /**
   * Set how close each joint must get to a waypoint before the arm moves on to the next. Paths planned from then on keep
   * this much following error away from the exclusion zones too.
   *
   * @param tolerance Path tolerance {@link Angle}, above 0.
   * @throws IllegalArgumentException if the tolerance is not above 0.
   * @see DoubleJointedArmConfig#withPathTolerance(Angle)
   */
  public void setPathTolerance(Angle tolerance) {
    m_planner.setTolerance(tolerance);
    m_pathTolerance = tolerance;
  }

  /**
   * Whether the arm can move straight between two sets of joint angles, each joint on its own, and stay clear of the
   * exclusion zones with the path tolerance to spare.
   *
   * @param fromLower Lower joint angle to start at, in radians.
   * @param fromUpper Upper joint angle to start at, in radians.
   * @param toLower   Lower joint angle to end at, in radians.
   * @param toUpper   Upper joint angle to end at, in radians.
   * @return True if the move is clear, or there are no exclusion zones.
   */
  public boolean isMoveClear(double fromLower, double fromUpper, double toLower, double toUpper) {
    return m_planner.isMoveClear(fromLower, fromUpper, toLower, toUpper);
  }

  /**
   * Change each joint's kG for where the arm is, so its {@link org.wpilib.math.controller.ArmFeedforward} holds the arm
   * against gravity: the shoulder carries the upper arm, and with the upper arm's angle from the lower arm the elbow's
   * gravity depends on its angle from horizontal. Each joint's kG is taken to be for the arm held straight out
   * horizontally, as configured or last live tuned, and is only sent to its motor controller again once it changes by
   * more than {@link DoubleJointedArmConfig#withKgUpdateThreshold(double) the threshold}. With the upper arm's angle
   * from horizontal kG never changes. Called by every command that moves the arm to an angle or position; call it each
   * loop when commanding the motor controllers directly.
   */
  public void updateKg() {
    updateKg(0, m_lowerSMC);
    updateKg(1, m_upperSMC);
  }

  /**
   * Change a joint's kG for where the arm is.
   *
   * @param joint 0 for the lower joint, 1 for the upper joint.
   * @param smc   The joint's {@link SmartMotorController}.
   */
  private void updateKg(int joint, SmartMotorController smc) {
    var armFeedforward = smc.getConfig().getArmFeedforward(smc.getClosedLoopControllerSlot());
    if (armFeedforward.isEmpty()) {
      return;
    }
    final double kG = armFeedforward.get().getKg();
    final double scale = m_dynamics.getKgScale(joint, getLowerAngle().in(Radians), getUpperAngle().in(Radians));
    if (Double.isNaN(m_kgSent[joint]) || kG != m_kgSent[joint]) {
      // First seen, or changed elsewhere, e.g. live tuned: it is the kG for where the arm was when it was last sent.
      final double sentScale = Double.isNaN(m_kgSent[joint]) || Double.isNaN(m_kgStraight[joint]) || m_kgStraight[joint] == 0 ? 1 : m_kgSent[joint] / m_kgStraight[joint];
      m_kgStraight[joint] = kG / sentScale;
      m_kgSent[joint] = kG;
    }
    if (Double.isNaN(scale)) {
      return;
    }
    final double compensated = m_kgStraight[joint] * scale;
    if (Math.abs(compensated - m_kgSent[joint]) > m_config.getKgUpdateThreshold() * Math.abs(m_kgStraight[joint])) {
      smc.setKg(compensated);
      m_kgSent[joint] = compensated;
    }
  }

  /**
   * Get a joint's kG, as last compensated for where the arm is.
   *
   * @param joint 0 for the lower joint, 1 for the upper joint.
   * @return kG in volts, or NaN without an ArmFeedforward.
   */
  public double getKg(int joint) {
    SmartMotorController smc = joint == 0 ? m_lowerSMC : m_upperSMC;
    return smc.getConfig().getArmFeedforward(smc.getClosedLoopControllerSlot()).map(feedforward -> feedforward.getKg()).orElse(Double.NaN);
  }

  /**
   * Get the equations of motion the simulation and kG compensation use.
   *
   * @return {@link DoubleJointedArmDynamics} of the arm.
   */
  public DoubleJointedArmDynamics getDynamics() {
    return m_dynamics;
  }

  /**
   * Get the arm's path follower, which every command that moves the arm to an angle or position uses.
   *
   * @return The arm's {@link DoubleJointedArmPathFollower}.
   */
  public DoubleJointedArmPathFollower getPathFollower() {
    return m_pathFollower;
  }

  /**
   * Get the arm's own telemetry, to apply its live tuning.
   *
   * @return The {@link DoubleJointedArmTelemetry}, or empty unless configured with
   *     {@link DoubleJointedArmConfig#withTelemetry(String, yams.core.telemetry.enums.TelemetryVerbosity)}.
   */
  public Optional<DoubleJointedArmTelemetry> getArmTelemetry() {
    return m_armTelemetry;
  }

  @Override
  public void updateTelemetry() {
    m_lowerSMC.updateTelemetry();
    m_upperSMC.updateTelemetry();
    m_armTelemetry.ifPresent(telemetry -> telemetry.publish(this));
    m_telemetry.updateLoopTime();
  }

  /**
   * Step the simulation once with both joints' voltages, then iterate both motor controllers' simulations.
   */
  @Override
  public void simIterate() {
    if (m_armSim.isPresent() && m_lowerSMC.getSimSupplier().isPresent() && m_upperSMC.getSimSupplier().isPresent()) {
      var lowerSupplier = m_lowerSMC.getSimSupplier().get();
      var upperSupplier = m_upperSMC.getSimSupplier().get();
      lowerSupplier.updateSimState();
      upperSupplier.updateSimState();
      m_armSim.get().update(m_lowerSMC.getConfig().getSimulationPeriod().in(Seconds));
      lowerSupplier.feedUpdateSim();
      upperSupplier.feedUpdateSim();
      m_lowerSMC.simIterate();
      m_upperSMC.simIterate();
      lowerSupplier.starveUpdateSim();
      upperSupplier.starveUpdateSim();
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
    var upperArmAngle = getUpperAngleFromHorizontal();
    var jointPos = getJoint(m_lowerArmLength, lowerArmAngle, m_lowerArmRootPos);
    m_lowerLigament.setAngle(lowerArmAngle.in(Degrees));
    m_upperLigament.setAngle(upperArmAngle.in(Degrees));
    m_upperRoot.setPosition(jointPos.getX(), jointPos.getY());
  }

  /**
   * Get the lower arm's pose, for publishing to AdvantageScope as a component of a robot model: where
   * it is mounted, as set in the lower arm's {@link MechanismPositionConfig}, turned to its angle.
   *
   * @return The lower arm's {@link Pose3d}, relative to the robot.
   * @see #getUpperPose3d()
   */
  @Override
  public Pose3d getPose3d() {
    return m_lowerArmConfig.getMechanismPositionConfig().getRotatedPose(getLowerAngle());
  }

  /**
   * Get the upper arm's pose, for publishing to AdvantageScope as a component of a robot model: at the
   * end of the lower arm, turned to the upper arm's angle from horizontal. Both angles are from the lower arm's
   * mount.
   *
   * @return The upper arm's {@link Pose3d}, relative to the robot.
   */
  public Pose3d getUpperPose3d() {
    MechanismPositionConfig mount = m_lowerArmConfig.getMechanismPositionConfig();
    Pose3d elbow = getPose3d().transformBy(new Transform3d(new Translation3d(m_lowerArmLength.in(Meters), 0, 0), Rotation3d.ZERO));
    if (mount.getMovementPlane() == Plane.YZ) {
      elbow = getPose3d().transformBy(new Transform3d(new Translation3d(0, m_lowerArmLength.in(Meters), 0), Rotation3d.ZERO));
    }
    return new Pose3d(elbow.getTranslation(), mount.getRotatedPose(getUpperAngleFromHorizontal()).getRotation());
  }

  @Override
  public String getName() {
    if (m_config.getTelemetryName().isPresent()) {
      return m_config.getTelemetryName().get();
    }
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
   * Get the shoulder's position setpoint.
   *
   * @return The shoulder's setpoint, or its angle without one.
   */
  public Angle getLowerSetpoint() {
    return m_lowerSMC.getMechanismPositionSetpoint().orElseGet(this::getLowerAngle);
  }

  /**
   * Get the elbow's position setpoint.
   *
   * @return The elbow's setpoint, or its angle without one.
   */
  public Angle getUpperSetpoint() {
    return m_upperSMC.getMechanismPositionSetpoint().orElseGet(this::getUpperAngle);
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
