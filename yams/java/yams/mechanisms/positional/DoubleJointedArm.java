package yams.mechanisms.positional;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Optional;
import java.util.function.Supplier;
import yams.exceptions.DoubleJointedArmConfigurationException;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.simulation.ArmSimSupplier;

import static edu.wpi.first.units.Units.*;

/**
 * Arm mechanism.
 */
public class DoubleJointedArm extends SmartPositionalMechanism
{

  /**
   * Upper arm {@link SmartMotorController}
   */
  private final SmartMotorController          m_upperSMC;
  /**
   * Lower arm {@link SmartMotorController}
   */
  private final SmartMotorController          m_lowerSMC;
  /**
   * Arm config.
   */
  private final ArmConfig                     m_lowerArmConfig;
  /**
   * Arm config.
   */
  private final ArmConfig                     m_upperArmConfig;
  /**
   * Simulation for the arm.
   */
  private       Optional<SingleJointedArmSim> m_lowerArmSim = Optional.empty();
  /**
   * Simulation for the arm.
   */
  private       Optional<SingleJointedArmSim> m_upperArmSim = Optional.empty();
  /**
   * Lower ligament
   */
  private       MechanismLigament2d           m_lowerLigament;
  /**
   * Upper root
   */
  private       MechanismRoot2d               m_upperRoot;
  /**
   * Upper ligament.
   */
  private       MechanismLigament2d           m_upperLigament;
  /**
   * Lower Arm root position in meters.
   */
  private       Translation2d                 m_lowerArmRootPos;
  /**
   * Lower arm length used for trig calculations on current position.
   */
  private       Distance                      m_lowerArmLength;
  /**
   * Upper arm length used for trig calculations on current position.
   */
  private       Distance                      m_upperArmLength;

  /**
   * Constructor for the Arm mechanism.
   *
   * @param lowerConfig Lower {@link ArmConfig} to use.
   */
  public DoubleJointedArm(ArmConfig lowerConfig, ArmConfig upperConfig)
  {
    m_lowerArmConfig = lowerConfig;
    m_upperArmConfig = upperConfig;
    m_lowerSMC = lowerConfig.getMotor();
    m_upperSMC = upperConfig.getMotor();
    if (lowerConfig.getMotor().getConfig().getSubsystem() != upperConfig.getMotor().getConfig().getSubsystem())
    {
      throw new DoubleJointedArmConfigurationException("SmartMotorControllers do not have the same subsystem!",
                                                       "Cannot create commands for single subsystem.",
                                                       "withSubsystem(this)");
    }
    m_subsystem = lowerConfig.getMotor().getConfig().getSubsystem();

    // Check that the starting angle is defined
    if (lowerConfig.getStartingAngle().isEmpty() || upperConfig.getStartingAngle().isEmpty())
    {
      throw new DoubleJointedArmConfigurationException("Arm starting angle is empty",
                                                       "Cannot create simulation.",
                                                       "withStartingPosition(Angle)");
    }

    // Check that the arm lengths are defined
    if (lowerConfig.getLength().isEmpty() || upperConfig.getLength().isEmpty())
    {
      throw new DoubleJointedArmConfigurationException(
          "Arm legnths must be defined to calculate current end position of the Double Jointed Arm!",
          "Cannot create mechanism",
          "withLength(Distance)");
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
    lowerConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.setupTelemetry(getName() + "/lower", m_lowerSMC);
    });
    upperConfig.getTelemetryName().ifPresent(name -> {
      m_telemetry.setupTelemetry(getName() + "/upper", m_upperSMC);
    });

    if (RobotBase.isSimulation())
    {

      if (lowerConfig.getLowerHardLimit().isEmpty() || upperConfig.getLowerHardLimit().isEmpty())
      {
        throw new DoubleJointedArmConfigurationException("Arm lower hard limit is empty",
                                                         "Cannot create simulation.",
                                                         "withHardLimit(Angle,Angle)");
      }
      if (lowerConfig.getUpperHardLimit().isEmpty() || upperConfig.getUpperHardLimit().isEmpty())
      {
        throw new DoubleJointedArmConfigurationException("Arm upper hard limit is empty",
                                                         "Cannot create simulation.",
                                                         "withHardLimit(Angle,Angle)");
      }

      // Setup Sim
      m_lowerArmSim = Optional.of(new SingleJointedArmSim(m_lowerSMC.getDCMotor(),
                                                          m_lowerSMC.getConfig().getGearing()
                                                                    .getMechanismToRotorRatio(),
                                                          lowerConfig.getMOI(),
                                                          lowerConfig.getLength().get().in(Meters),
                                                          lowerConfig.getLowerHardLimit().get().in(Radians),
                                                          lowerConfig.getUpperHardLimit().get().in(Radians),
                                                          true,
                                                          lowerConfig.getStartingAngle().get().in(Radians),
                                                          0.002 / 4096.0,
                                                          0.0));// Add noise with a std-dev of 1 tick
      m_lowerSMC.setSimSupplier(new ArmSimSupplier(m_lowerArmSim.get(), m_lowerSMC));
      m_upperArmSim = Optional.of(new SingleJointedArmSim(m_upperSMC.getDCMotor(),
                                                          m_upperSMC.getConfig().getGearing()
                                                                    .getMechanismToRotorRatio(),
                                                          upperConfig.getMOI(),
                                                          m_upperArmLength.in(Meters),
                                                          upperConfig.getLowerHardLimit().get().in(Radians),
                                                          upperConfig.getUpperHardLimit().get().in(Radians),
                                                          true,
                                                          upperConfig.getStartingAngle().get().in(Radians),
                                                          0.002 / 4096.0,
                                                          0.0));// Add noise with a std-dev of 1 tick
      m_upperSMC.setSimSupplier(new ArmSimSupplier(m_upperArmSim.get(), m_upperSMC));

      var lowerStartingAngle = lowerConfig.getStartingAngle().get();
      var upperStartingAngle = upperConfig.getStartingAngle().get();

      var upperArmRootPos = getJoint(m_lowerArmLength, lowerStartingAngle, m_lowerArmRootPos);

      var windowX =
          lowerMechPosCfg.getWindowXDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowXDimension(
              m_upperArmLength).in(Meters);
      var windowY =
          lowerMechPosCfg.getWindowYDimension(m_lowerArmLength).in(Meters) + upperMechPosCfg.getWindowYDimension(
              m_upperArmLength).in(Meters);

      m_mechanismWindow = new Mechanism2d(windowX, windowY);
      m_mechanismRoot = m_mechanismWindow.getRoot("Lower Root", m_lowerArmRootPos.getX(), m_lowerArmRootPos.getY());
      m_lowerLigament = m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(" lower",
                                                                                             m_lowerArmLength.in(Meters),
                                                                                             lowerStartingAngle.in(
                                                                                                 Degrees),
                                                                                             7,
                                                                                             lowerConfig.getSimColor()));
      m_upperRoot = m_mechanismWindow.getRoot("Upper Root", upperArmRootPos.getX(), upperArmRootPos.getY());
      m_upperLigament = m_upperRoot.append(new MechanismLigament2d("upper",
                                                                   m_upperArmLength.in(Meters),
                                                                   upperStartingAngle.in(Degrees),
                                                                   6,
                                                                   upperConfig.getSimColor()));
      SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);

      m_upperSMC.setupSimulation();
      m_lowerSMC.setupSimulation();
    }
    // Apply configs
    lowerConfig.applyConfig();
    upperConfig.applyConfig();
  }

  /**
   * Get the joint {@link Translation2d} of the arm in Meters.
   *
   * @param armLen   {@link Distance} length of the arm.
   * @param armAngle {@link Angle} angle of the arm.
   * @param offset   {@link Translation2d} root position to offset by.
   * @return {@link Translation2d} of the joint.
   */
  private Translation2d getJoint(Distance armLen, Angle armAngle, Translation2d offset)
  {
    return new Translation2d(armLen.times(Math.cos(armAngle.in(Radians))).in(Meters),
                             armLen.times(Math.sin(armAngle.in(Radians))).in(Meters)).plus(offset);
  }

  /**
   * Get the {@link Translation2d} of the double jointed arm.
   *
   * @return {@link Translation2d} of the double jointed arm.
   */
  public Translation2d getTranslation()
  {
    return getJoint(m_upperArmLength,
                    m_upperSMC.getMechanismPosition(),
                    getJoint(m_lowerArmLength, m_lowerSMC.getMechanismPosition(), Translation2d.kZero));
  }

  public enum ElbowRequest
  {
    UP, DOWN
  }

  public Command setTranslation(Translation2d translation, ElbowRequest elbowRequest)
  {
    var distance = Meters.of(translation.getNorm());

    if (m_lowerArmLength.minus(m_upperArmLength).abs(Meters) <= distance.in(Meters) &&
        distance.in(Meters) <= m_lowerArmLength.plus(m_upperArmLength).in(Meters))
    {
      var elbowAngle = Radians.of(Math.acos((Math.pow(translation.getX(), 2) + Math.pow(translation.getX(), 2) -
                        Math.pow(m_lowerArmLength.in(Meters), 2) - Math.pow(m_upperArmLength.in(Meters), 2)) /
                       (2 * m_lowerArmLength.in(Meters) * m_upperArmLength.in(Meters))));
      if(elbowRequest == ElbowRequest.UP)
        elbowAngle = elbowAngle.times(-1);

    }
    return null;
  }

  @Override
  public void updateTelemetry()
  {
    m_lowerSMC.updateTelemetry();
    m_upperSMC.updateTelemetry();
  }

  @Override
  public void simIterate()
  {
    if (m_lowerArmSim.isPresent() && m_lowerSMC.getSimSupplier().isPresent() && m_upperArmSim.isPresent() &&
        m_upperSMC.getSimSupplier().isPresent())
    {
      m_lowerSMC.getSimSupplier().get().updateSimState();
      m_lowerSMC.simIterate();
      m_lowerSMC.getSimSupplier().get().starveUpdateSim();
      m_upperSMC.getSimSupplier().get().updateSimState();
      m_upperSMC.simIterate();
      m_upperSMC.getSimSupplier().get().starveUpdateSim();
      RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(m_lowerArmSim.get().getCurrentDrawAmps(),
                                                                               m_upperArmSim.get()
                                                                                            .getCurrentDrawAmps()));
      visualizationUpdate();
    }
  }

  /**
   * Updates the mechanism ligament with the current angle of the arm.
   *
   * @see SmartPositionalMechanism#visualizationUpdate()
   */
  @Override
  public void visualizationUpdate()
  {
    var lowerArmAngle = m_lowerSMC.getMechanismPosition();
    var upperArmAngle = m_upperSMC.getMechanismPosition();
    var jointPos      = getJoint(m_lowerArmLength, lowerArmAngle, m_lowerArmRootPos);
    m_lowerLigament.setAngle(lowerArmAngle.in(Degrees));
    m_upperLigament.setAngle(upperArmAngle.in(Degrees));
    m_upperRoot.setPosition(jointPos.getX(), jointPos.getY());
  }

  /**
   * Get the relative position of the mechanism, taking into account the relative position defined in the
   * {@link MechanismPositionConfig}.
   *
   * @return The relative position of the mechanism as a {@link Translation3d}.
   */
  @Override
  public Translation3d getRelativeMechanismPosition()
  {
    return null;
  }

  @Override
  public String getName()
  {
    return "DoubleJointedArm_" + m_lowerArmConfig.getTelemetryName().orElse("Lower") + "_" +
           m_upperArmConfig.getTelemetryName().orElse("Upper");
  }

  @Override
  public Trigger max()
  {
    return null;
  }

  // TODO Probably getting replaced with joint requests.
  public Angle getLowerAngle() {
    return m_lowerSMC.getMechanismPosition();
  }

  public Command setAngle(Angle lowerAngle,  Angle upperAngle) {
    return Commands.run(() -> {
      if (lowerAngle != null) {m_lowerSMC.setPosition(lowerAngle);}
      if (upperAngle != null) {m_upperSMC.setPosition(upperAngle);}
    }, m_subsystem).withName(m_subsystem.getName() + " SetAngle");
  }

  public Command set(Double lowerDutycycle, Double upperDutycycle) {
    return Commands.startRun(() -> {
      if (lowerDutycycle != null) {m_lowerSMC.stopClosedLoopController();}
      if (upperDutycycle != null) {m_upperSMC.stopClosedLoopController();}
      }, () -> {
      if (lowerDutycycle != null) {m_lowerSMC.setDutyCycle(lowerDutycycle);}
      if (upperDutycycle != null) {m_upperSMC.setDutyCycle(upperDutycycle);}
      }, m_subsystem).finallyDo(() -> {
        if (lowerDutycycle != null) {m_lowerSMC.startClosedLoopController();}
        if (upperDutycycle != null) {m_upperSMC.startClosedLoopController();}
      }).withName(m_subsystem.getName() + " SetDutyCycle");
  }

  @Override
  public Command set(double dutycycle)
  {
    return null;
  }

  @Override
  public Command set(Supplier<Double> dutycyle)
  {
    return null;
  }

  @Override
  public Command setVoltage(Voltage volts)
  {
    return null;
  }

  @Override
  public Command setVoltage(Supplier<Voltage> volts)
  {
    return null;
  }

  @Override
  public Trigger min()
  {
    return null;
  }

  @Override
  public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration)
  {
    return null;
  }

}