package frc.robot.subsystems.turretakit;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.littletonrobotics.junction.Logger;

import frc.robot.constants.SubsystemConstants;
import frc.robot.util.PhoenixUtil;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXWrapper;
import yams.telemetry.SmartMotorControllerTelemetryConfig;
import yams.units.CRTAbsoluteEncoder;
import yams.units.CRTAbsoluteEncoderConfig;

/**
 * Example of wiring a YAMS CRTAbsoluteEncoder with two CANcoders on a turret.
 *
 * <p>How to use:
 *
 * <ul>
 *   <li>Describe the encoder gearing in {@link #buildCrtConfig()} and set mechanism
 *       range/tolerance for your turret.
 *       <ul>
 *         <li>If both encoders share a drive gear, use
 *             {@link CRTAbsoluteEncoderConfig#withCommonDriveGear(double, int, int, int)} as
 *             shown.</li>
 *         <li>If they have separate gear trains, use
 *             {@link CRTAbsoluteEncoderConfig#withAbsoluteEncoder1Gearing(int...)} /
 *             {@link CRTAbsoluteEncoderConfig#withAbsoluteEncoder2Gearing(int...)} for simple
 *             mesh chains, or the {@code ...GearingStages(...)} helpers for explicit driver/driven
 *             pairs.</li>
 *       </ul>
 *   <li>AdvantageKit logs under "Turret/CRT" keys for solver status/error/iterations
 *       and gear recommender info.
 * </ul>
 */
public class TurretWithYAMSCRT {
  // Hardware/config
  private final TalonFX turretMotor;
  private final CANcoder cancoderA;
  private final CANcoder cancoderB;
  private final StatusSignal<Angle> absPositionASignal;
  private final StatusSignal<Angle> absPositionBSignal;
  private final SmartMotorController motor;
  private final Pivot turret;
  private final CRTAbsoluteEncoderConfig crtConfig;
  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig;

  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private String lastSeedStatus = "NOT_ATTEMPTED";

  public TurretWithYAMSCRT(Subsystem subsystem) {  

    turretMotor = new TalonFX(1, "rio");
    cancoderA = new CANcoder(2, "rio");
    cancoderB = new CANcoder(3, "rio");

    var cfgA = new CANcoderConfiguration();
    cfgA.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cfgA.MagnetSensor.MagnetOffset = 0;
    cancoderA.getConfigurator().apply(cfgA);

    var cfgB = new CANcoderConfiguration();
    cfgB.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    cfgB.MagnetSensor.MagnetOffset = 0;
    cancoderB.getConfigurator().apply(cfgB);

    absPositionASignal = cancoderA.getAbsolutePosition();
    absPositionBSignal = cancoderB.getAbsolutePosition();

    motorTelemetryConfig =
        new SmartMotorControllerTelemetryConfig()
            .withMechanismPosition()
            .withRotorPosition()
            .withRotorVelocity()
            .withMechanismLowerLimit()
            .withMechanismUpperLimit();

    SmartMotorControllerConfig motorConfig =
        new SmartMotorControllerConfig(this)
            .withClosedLoopController(
                4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSimClosedLoopController(
                130, 0, 3.4, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1500))
            .withSoftLimit(Degrees.of(0), Degrees.of(500))
            .withGearing(
                new MechanismGearing(
                    GearBox.fromReductionStages(
                        12/50,
                        10/90)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotorV2", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withControlMode(ControlMode.CLOSED_LOOP);

    motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    MechanismPositionConfig robotToMechanism =
        new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(
                new Translation3d(
                    Meters.of(0.0),
                    Meters.of(0.0),
                    Meters.of(0.451739)
                    ));

    PivotConfig pivotConfig =
        new PivotConfig(motor)
            .withHardLimit(Degrees.of(0), Degrees.of(720))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(Degrees.of(0))
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(0.2);

    turret = new Pivot(pivotConfig);
    crtConfig = buildCrtConfig();
    logCrtConfigTelemetry();
  }

  @Override
  public Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  @Override
  public Angle getAngle() {
    return turret.getAngle();
  }

  @Override
  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    Logger.recordOutput("Turret/CRT/ManualRerunTimestampSec", edu.wpi.first.wpilibj.Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  @Override
  public void periodic() {
    if (!rotorSeededFromAbs) {
      attemptRotorSeedFromCANCoders();
    }
    turret.updateTelemetry();
  }

  private void attemptRotorSeedFromCANCoders() {
    AbsSensorRead absRead = readAbsSensors();
    if (!absRead.ok()) {
      lastSeedStatus = absRead.status();
      return;
    }

    lastAbsA = absRead.absA();
    lastAbsB = absRead.absB();

    var solver = new CRTAbsoluteEncoder(crtConfig);
    var solvedAngle = solver.getAngleOptional();

    Logger.recordOutput("Turret/CRT/AbsA", lastAbsA);
    Logger.recordOutput("Turret/CRT/AbsB", lastAbsB);
    Logger.recordOutput("Turret/CRT/SolverStatus", solver.getLastStatus());
    Logger.recordOutput("Turret/CRT/SolverErrorRot", solver.getLastErrorRotations());
    Logger.recordOutput("Turret/CRT/SolverIterations", solver.getLastIterations());

    if (solvedAngle.isEmpty()) {
      Logger.recordOutput("Turret/CRT/SolutionFound", false);
      lastSeedStatus = solver.getLastStatus();
      return;
    }

    double turretRotations = solvedAngle.get().in(Rotations);
    double rotorRotations = turretRotations * SubsystemConstants.TURRET_MOTOR_TO_TURRET_RATIO;
    StatusCode seedStatus = turretMotor.setPosition(turretRotations, 0.25);
    if (!seedStatus.isOK()) {
      seedStatus = PhoenixUtil.tryUntilOk(4, () -> turretMotor.setPosition(turretRotations, 0.25));
    }

    if (seedStatus.isOK()) {
      rotorSeededFromAbs = true;
      lastSeededTurretDeg = turretRotations * 360.0;
      lastSeedError = solver.getLastErrorRotations();
      Logger.recordOutput("Turret/CRT/SolutionFound", true);
      Logger.recordOutput("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
      Logger.recordOutput("Turret/CRT/SeededRotorRot", rotorRotations);
      Logger.recordOutput("Turret/CRT/MatchErrorRot", lastSeedError);
    }

    lastSeedStatus = seedStatus.toString();
    Logger.recordOutput("Turret/CRT/SeedStatus", seedStatus.toString());
    Logger.recordOutput("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  private AbsSensorRead readAbsSensors() {
    boolean haveDevices = cancoderA != null && cancoderB != null;
    if (haveDevices) {
      var status = BaseStatusSignal.refreshAll(absPositionASignal, absPositionBSignal);
      if (status.isOK()) {
        return new AbsSensorRead(
            true,
            absPositionASignal.getValue().in(Rotations),
            absPositionBSignal.getValue().in(Rotations),
            status.toString());
      }
    }
    return new AbsSensorRead(false, Double.NaN, Double.NaN, "NO_DEVICES");
  }

  private Angle readAbsoluteEncoder(StatusSignal<Angle> signal) {
    Angle value = signal != null ? signal.getValue() : null;
    return value != null ? value : Rotations.of(Double.NaN);
  }

  /**
   * Build the CRT configuration: supply absolute encoder readings, gearing definition, offsets,
   * mechanism travel, and match tolerance. Adjust these to match your turret: if you don't have a
   * common drive stage, use the gearing helpers instead of {@link #withCommonDriveGear(double, int,
   * int, int)}.
   */
  private CRTAbsoluteEncoderConfig buildCrtConfig() {
    // Example gearing: 50T drives encoder pinions 34T and 33T via common drive stage of 9:1
    return new CRTAbsoluteEncoderConfig(
            () -> readAbsoluteEncoder(absPositionASignal),
            () -> readAbsoluteEncoder(absPositionBSignal))
        .withCommonDriveGear(
          90/10, 
          50, 
          34, 
          33)
        .withMechanismRange(Rotations.of(0.0), Rotations.of(2.0))
        .withMatchTolerance(Rotations.of(0.05))
        .withCrtGearRecommendationConstraints(
          1.2, 
          15, 
          60, 
          40);
  }

  private void logCrtConfigTelemetry() {
    Logger.recordOutput(
        "Turret/CRT/Config/RatioA", crtConfig.getEncoder1RotationsPerMechanismRotation());
    Logger.recordOutput(
        "Turret/CRT/Config/RatioB", crtConfig.getEncoder2RotationsPerMechanismRotation());
    Logger.recordOutput(
        "Turret/CRT/Config/UniqueCoverageRot",
        crtConfig.getUniqueCoverageRotations().orElse(Double.NaN));
    Logger.recordOutput(
        "Turret/CRT/Config/CoverageSatisfiesRange",
        crtConfig
            .getUniqueCoverageRotations()
            .map(coverage -> coverage >= 2.0)
            .orElse(false));

    var configPair = crtConfig.getRecommendedCrtGearPair();
    Logger.recordOutput("Turret/CRT/Config/RecommendedPairFound", configPair.isPresent());
    if (configPair.isPresent()) {
      var pair = configPair.get();
      Logger.recordOutput("Turret/CRT/Config/Recommender/RecommendedGearA", pair.gearA());
      Logger.recordOutput("Turret/CRT/Config/Recommender/RecommendedGearB", pair.gearB());
      Logger.recordOutput(
          "Turret/CRT/Config/Recommender/RecommendedCoverageRot", pair.coverageRot());
      Logger.recordOutput(
          "Turret/CRT/Config/Recommender/RecommendedIterations", pair.theoreticalIterations());
    }
  }

  private static record AbsSensorRead(boolean ok, double absA, double absB, String status) {}
}
