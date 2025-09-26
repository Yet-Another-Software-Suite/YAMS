package yams.mechanisms.positional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import yams.exceptions.DifferentialMechanismConfigurationException;
import yams.mechanisms.config.DifferentialMechanismConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.simulation.DCMotorSimSupplier;

import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;

/**
 * Arm mechanism.
 */
public class DifferentialMechanism extends SmartPositionalMechanism {

    /**
     * Left {@link SmartMotorController}
     */
    private final SmartMotorController m_leftSMC;
    /**
     * Right {@link SmartMotorController}
     */
    private final SmartMotorController m_rightSMC;
    /**
     * Differential Mechanism config.
     */
    private final DifferentialMechanismConfig m_config;
    /**
     * Simulation for the left motor.
     */
    private Optional<DCMotorSim> m_leftSim = Optional.empty();
    /**
     * Simulation for the right motor.
     */
    private Optional<DCMotorSim> m_rightSim = Optional.empty();
    /**
     * Twist Ligament
     */
    private MechanismLigament2d m_twistLigament;
    /**
     * Twist root
     */
    private MechanismRoot2d m_twistRoot;
    /**
     * Arm length.
     */
    private Distance m_armLength;
    /**
     * Tilt root.
     */
    private Translation2d m_tiltRoot;

    /**
     * Constructor for the Differential mechanism.
     *
     * @param diffConfig Lower {@link DifferentialMechanismConfig} to use.
     */
    public DifferentialMechanism(DifferentialMechanismConfig diffConfig) {
        m_config = diffConfig;
        m_leftSMC = diffConfig.getLeftMotorController();
        m_rightSMC = diffConfig.getRightMotorController();

        if (m_leftSMC.getConfig().getSubsystem() != m_rightSMC.getConfig().getSubsystem()) {
            throw new DifferentialMechanismConfigurationException("SmartMotorControllers do not have the same subsystem!",
                    "Cannot create commands for single subsystem.",
                    "withSubsystem(this)");
        }
        m_subsystem = m_leftSMC.getConfig().getSubsystem();

        // Check that the starting angle is defined
        if (m_config.getStartingTiltAngle().isEmpty() || m_config.getStartingTwistAngle().isEmpty()) {
            throw new DifferentialMechanismConfigurationException("Starting angle is empty",
                    "Cannot create simulation.",
                    "withTwistStartingPosition(Angle) OR DifferentialMechanismConfig.withTiltStartingPosition(Angle)");
        }

        // Check that the arm lengths are defined
        if (m_config.getLength().isEmpty()) {
            throw new DifferentialMechanismConfigurationException(
                    "Lengths must be defined to calculate current end position of the DifferentialMechanism!",
                    "Cannot create mechanism",
                    "withLength(Distance)");
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
            m_telemetry.setupTelemetry(getName() + "/left", m_leftSMC);
            m_telemetry.setupTelemetry(getName() + "/right", m_rightSMC);
        });

        if (RobotBase.isSimulation()) {
            var startingtilt = m_config.getStartingTiltAngle().get();
            var startingtwist = m_config.getStartingTwistAngle().get();
            var startingleft = m_config.getLeftMechanismPosition(startingtilt, startingtwist);
            var startingright = m_config.getRightMechanismPosition(startingtilt, startingtwist);
            m_tiltRoot = new Translation2d(m_armLength.in(Meters), 0);

            // Setup Sim
            m_leftSim = Optional.of(new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(m_leftSMC.getDCMotor(),
                            m_config.getMOI(),
                            m_leftSMC.getConfig().getGearing().getMechanismToRotorRatio()),
                    m_leftSMC.getDCMotor()));
            m_rightSim = Optional.of(new DCMotorSim(
                    LinearSystemId.createDCMotorSystem(m_rightSMC.getDCMotor(),
                            m_config.getMOI(),
                            m_rightSMC.getConfig().getGearing().getMechanismToRotorRatio()),
                    m_rightSMC.getDCMotor()));
            m_leftSMC.setSimSupplier(new DCMotorSimSupplier(m_leftSim.get(), m_leftSMC));
            m_rightSMC.setSimSupplier(new DCMotorSimSupplier(m_rightSim.get(), m_rightSMC));

            m_mechanismWindow = new Mechanism2d(mechPosCfg.getWindowXDimension(m_armLength).plus(Inches.of(4)).in(Meters),
                    mechPosCfg.getWindowYDimension(m_armLength.plus(Inches.of(4))).in(Meters));
            m_mechanismRoot = m_mechanismWindow.getRoot("Root", m_tiltRoot.getX(), m_tiltRoot.getY());
            m_mechanismLigament = m_mechanismRoot.append(new MechanismLigament2d(" tilt",
                    m_armLength.in(Meters),
                    startingtilt.in(Degrees),
                    7,
                    m_config.getSimColor()));
            m_twistRoot = m_mechanismWindow.getRoot("Twist Root", m_armLength.in(Meters), m_armLength.in(Meters));
            m_twistLigament = m_twistRoot.append(new MechanismLigament2d(" twist",
                    Inches.of(4).in(Meters),
                    startingtwist.in(Degrees),
                    6,
                    new Color8Bit(Color.kRed)));

            SmartDashboard.putData(getName() + "/mechanism", m_mechanismWindow);

            m_leftSMC.setupSimulation();
            m_rightSMC.setupSimulation();

            m_leftSim.get().setAngle(startingleft.in(Radians));
            m_rightSim.get().setAngle(startingright.in(Radians));
        }

        // Apply configs
        m_config.applyConfig();
    }


    /**
     * Get the twist {@link Angle} of the mechanism.
     *
     * @return Twist {@link Angle}.
     */
    public Angle getTwistPosition() {
        if (m_config.getTwistAngleSupplier().isPresent())
            return m_config.getTwistAngleSupplier().get().get();
        return m_config.getTwistAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
    }

    /**
     * Get the tilt {@link Angle} of the mechanism.
     *
     * @return Tilt {@link Angle}.
     */
    public Angle getTiltPosition() {
        if (m_config.getTiltAngleSupplier().isPresent())
            return m_config.getTiltAngleSupplier().get().get();
        return m_config.getTiltAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
    }

    /**
     * Set the position of the differential mechanism using suppliers.
     *
     * @param tilt  Tilt of the Differential Mechanism.
     * @param twist Twist of the Differential Mechanism.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand} to set the position.
     */
    public Command setPosition(Supplier<Angle> tilt, Supplier<Angle> twist) {
        return Commands.run(() -> {
            var left = m_config.getLeftMechanismPosition(tilt.get(), twist.get());
            var right = m_config.getRightMechanismPosition(tilt.get(), twist.get());
            m_leftSMC.setPosition(left);
            m_rightSMC.setPosition(right);
        }, m_subsystem).withName(getName() + " set position");
    }

    /**
     * Set the dutycycle of the differential mechanism.
     *
     * @param twist Twist dutycycle.
     * @param tilt  Tilt dutycycle.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand} to set the differential mechanism duty cycle.
     */
    public Command set(double twist, double tilt) {
        return Commands.startRun(()->{
            m_leftSMC.stopClosedLoopController();
            m_rightSMC.stopClosedLoopController();
        },() -> {
            var left = tilt - twist;
            var right = tilt + twist;
            m_leftSMC.setDutyCycle(left);
            m_rightSMC.setDutyCycle(right);
        }, m_subsystem).finallyDo(()->{
            m_leftSMC.startClosedLoopController();
            m_rightSMC.startClosedLoopController();
        }).withName(getName() + " set dutycycle");
    }

    /**
     * Set the position of the differential mechanism.
     *
     * @param tilt  Tilt of the differential mechanism.
     * @param twist Twist of the differential mechanism.
     * @return {@link edu.wpi.first.wpilibj2.command.RunCommand} to set the position.
     */
    public Command setPosition(Angle tilt, Angle twist) {
        return Commands.run(() -> {
            var left = m_config.getLeftMechanismPosition(tilt, twist);
            var right = m_config.getRightMechanismPosition(tilt, twist);
            m_leftSMC.setPosition(left);
            m_rightSMC.setPosition(right);
        }, m_subsystem).withName(getName() + " set position");
    }

    @Override
    public void updateTelemetry() {
        m_leftSMC.updateTelemetry();
        m_rightSMC.updateTelemetry();
    }

    @Override
    public void simIterate() {
        if (m_leftSim.isPresent() && m_leftSMC.getSimSupplier().isPresent() && m_rightSim.isPresent() &&
                m_rightSMC.getSimSupplier().isPresent()) {
            m_leftSMC.getSimSupplier().get().updateSimState();
            m_leftSMC.simIterate();
            m_leftSMC.getSimSupplier().get().starveUpdateSim();
            m_rightSMC.getSimSupplier().get().updateSimState();
            m_rightSMC.simIterate();
            m_rightSMC.getSimSupplier().get().starveUpdateSim();
            RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
                    m_leftSim.get().getCurrentDrawAmps(),
                    m_rightSim.get().getCurrentDrawAmps()));
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
        var twistAngle = m_config.getTwistAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
        var tiltAngle = m_config.getTiltAngle(m_leftSMC.getMechanismPosition(), m_rightSMC.getMechanismPosition());
        var twistRoot = new Translation2d(m_armLength.in(Meters), Rotation2d.fromDegrees(tiltAngle.in(Degrees))).plus(m_tiltRoot);
        m_mechanismLigament.setAngle(tiltAngle.in(Degrees));

        m_twistRoot.setPosition(twistRoot.getX(), twistRoot.getY());
        m_twistLigament.setAngle(twistAngle.in(Degrees));
    }

    /**
     * Get the relative position of the mechanism, taking into account the relative position defined in the
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

    @Override
    @Deprecated
    public Trigger max() {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Command set(double dutycycle) {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Command set(Supplier<Double> dutycyle) {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Command setVoltage(Voltage volts) {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Command setVoltage(Supplier<Voltage> volts) {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Trigger min() {
        throw new RuntimeException("Unimplemented");
    }

    @Override
    @Deprecated
    public Command sysId(Voltage maximumVoltage, Velocity<VoltageUnit> step, Time duration) {
        throw new RuntimeException("Unimplemented");
    }

}