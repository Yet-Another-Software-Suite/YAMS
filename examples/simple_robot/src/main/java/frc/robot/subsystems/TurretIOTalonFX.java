package frc.robot.subsystems.turretakit;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Second;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.CANcoderSimState;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.SubsystemConstants;
import frc.robot.util.PhoenixUtil;
import java.util.Objects;
import org.littletonrobotics.junction.Logger;
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

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX turretMotor;
  private final SmartMotorControllerTelemetryConfig motorTelemetryConfig;
  private final SmartMotorControllerConfig motorConfig;
  private final SmartMotorController motor;
  private final MechanismPositionConfig robotToMechanism;
  private final PivotConfig pivotConfig;
  private final Pivot turret;
  private final CANcoder cancoderA;
  private final CANcoder cancoderB;
  private final CANcoderSimState cancoderSimA;
  private final CANcoderSimState cancoderSimB;
  private final StatusSignal<Voltage> appliedVoltsSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final Subsystem subsystem;
  private double lastSimTurretRotations = Double.NaN;
  private double lastSimTimestampSec = Double.NaN;
  private boolean rotorSeededFromAbs = false;
  private double lastSeededTurretDeg = Double.NaN;
  private double lastSeedError = Double.NaN;
  private double lastAbsA = Double.NaN;
  private double lastAbsB = Double.NaN;
  private boolean lastUsingComputedSimSignals = false;
  private String lastSeedStatus = "NOT_ATTEMPTED";

  public TurretIOTalonFX(Subsystem subsystem) {
    this.subsystem = Objects.requireNonNull(subsystem, "subsystem");

    turretMotor = new TalonFX(SubsystemConstants.TurretID, SubsystemConstants.RIOCANBUS);

    cancoderA = new CANcoder(SubsystemConstants.TURRET_CANCODER_A_ID, SubsystemConstants.RIOCANBUS);

    cancoderB = new CANcoder(SubsystemConstants.TURRET_CANCODER_B_ID, SubsystemConstants.RIOCANBUS);

    var cancoderConfigurationA = new CANcoderConfiguration();
    cancoderConfigurationA.MagnetSensor.MagnetOffset = SubsystemConstants.TURRET_ABS_A_OFFSET_ROT;
    cancoderA.getConfigurator().apply(cancoderConfigurationA);

    var cancoderConfigurationB = new CANcoderConfiguration();
    cancoderConfigurationB.MagnetSensor.MagnetOffset = SubsystemConstants.TURRET_ABS_B_OFFSET_ROT;
    cancoderB.getConfigurator().apply(cancoderConfigurationB);

    cancoderSimA = cancoderA.getSimState();
    cancoderSimB = cancoderB.getSimState();
    cancoderSimA.setSupplyVoltage(12.0);
    cancoderSimB.setSupplyVoltage(12.0);

    appliedVoltsSignal = turretMotor.getMotorVoltage();
    statorCurrentSignal = turretMotor.getStatorCurrent();
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, appliedVoltsSignal, statorCurrentSignal);

    motorTelemetryConfig =
        new SmartMotorControllerTelemetryConfig()
            .withMechanismPosition()
            .withRotorPosition()
            .withRotorVelocity()
            .withMechanismLowerLimit()
            .withMechanismUpperLimit();

    motorConfig =
        new SmartMotorControllerConfig(this.subsystem)
            .withClosedLoopController(
                4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            .withSimClosedLoopController(
                130, 0, 3.4, DegreesPerSecond.of(1000), DegreesPerSecondPerSecond.of(1500))
            .withSoftLimit(Degrees.of(0), Degrees.of(500))
            .withGearing(
                new MechanismGearing(
                    GearBox.fromReductionStages(
                        SubsystemConstants.TURRET_STAGE1_RATIO,
                        SubsystemConstants.TURRET_STAGE2_RATIO)))
            .withIdleMode(MotorMode.BRAKE)
            .withTelemetry("TurretMotorV2", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withControlMode(ControlMode.CLOSED_LOOP);

    motor = new TalonFXWrapper(turretMotor, DCMotor.getKrakenX60(1), motorConfig);

    robotToMechanism =
        new MechanismPositionConfig()
            .withMaxRobotHeight(Meters.of(1.5))
            .withMaxRobotLength(Meters.of(0.75))
            .withRelativePosition(
                new Translation3d(
                    Meters.of(-0.1524), // back from robot center
                    Meters.of(0.0), // centered left/right
                    Meters.of(0.451739) // up from the floor reference
                    ));

    pivotConfig =
        new PivotConfig(motor)
            .withHardLimit(Degrees.of(0), Degrees.of(450))
            .withTelemetry("Turret", TelemetryVerbosity.HIGH)
            .withStartingPosition(Degrees.of(0))
            .withMechanismPositionConfig(robotToMechanism)
            .withMOI(0.2);

    turret = new Pivot(pivotConfig);
    logCrtCoverage();
  }

  @Override
  public edu.wpi.first.wpilibj2.command.Command setAngle(Angle angle) {
    return turret.setAngle(angle);
  }

  @Override
  public Angle getAngle() {
    return turret.getAngle();
  }

  @Override
  public double getRobotRelativeYawRadians() {
    return getAngle().in(edu.wpi.first.units.Units.Radians);
  }

  @Override
  public void rerunCrtSeed() {
    rotorSeededFromAbs = false;
    Logger.recordOutput("Turret/CRT/ManualRerunTimestampSec", Timer.getFPGATimestamp());
    attemptRotorSeedFromCANCoders();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.turretAngle = new Rotation2d(getAngle().in(Radians));
    AngularVelocity currentVelocity = motor.getMechanismVelocity();
    inputs.yawVelocityRadPerSec = currentVelocity.in(Radians.per(Second));

    StatusCode motorStatus = BaseStatusSignal.refreshAll(appliedVoltsSignal, statorCurrentSignal);
    inputs.appliedVolts = appliedVoltsSignal.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrentSignal.getValueAsDouble();

    inputs.absARot = lastAbsA;
    inputs.absBRot = lastAbsB;
    inputs.usingComputedSimSignals = lastUsingComputedSimSignals;
    inputs.seedStatus = Objects.toString(lastSeedStatus, "NOT_ATTEMPTED");
    inputs.seeded = rotorSeededFromAbs;
    inputs.seededTurretDeg = lastSeededTurretDeg;
    inputs.seedErrorRot = lastSeedError;
    inputs.connected = motorStatus.isOK();
  }

  @Override
  public void periodic() {
    if (!rotorSeededFromAbs) {
      attemptRotorSeedFromCANCoders();
    }
    turret.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    turret.simIterate();
    updateCancoderSimState();
  }

  private void attemptRotorSeedFromCANCoders() {
    // CANcoder rotations per turret rotation; derived from stage2 ratio and 50T->(27/31T) gear ratio.
    double ratioA = cancoderRatio(SubsystemConstants.TURRET_ABS_GEAR_A_TEETH);
    double ratioB = cancoderRatio(SubsystemConstants.TURRET_ABS_GEAR_B_TEETH);

    double absA;
    double absB;
    boolean usingComputedSimSignals = false;

    boolean haveDevices = cancoderA != null && cancoderB != null;
    if (haveDevices) {
      var absSignalA = cancoderA.getAbsolutePosition();
      var absSignalB = cancoderB.getAbsolutePosition();
      var status = BaseStatusSignal.refreshAll(absSignalA, absSignalB);
      if (status.isOK()) {
        absA = wrap01(absSignalA.getValueAsDouble());
        absB = wrap01(absSignalB.getValueAsDouble());
      } else if (Constants.currentMode == Constants.Mode.SIM) {
        double turretRot = getAngle().in(Degrees) / 360.0;
        absA = wrap01(ratioA * turretRot + SubsystemConstants.TURRET_ABS_A_OFFSET_ROT);
        absB = wrap01(ratioB * turretRot + SubsystemConstants.TURRET_ABS_B_OFFSET_ROT);
        usingComputedSimSignals = true;
      } else {
        Logger.recordOutput("Turret/CRT/SeedStatus", status.toString());
        lastSeedStatus = status.toString();
        return;
      }
    } else if (Constants.currentMode == Constants.Mode.SIM) {
      double turretRot = getAngle().in(Degrees) / 360.0;
      absA = wrap01(ratioA * turretRot + SubsystemConstants.TURRET_ABS_A_OFFSET_ROT);
      absB = wrap01(ratioB * turretRot + SubsystemConstants.TURRET_ABS_B_OFFSET_ROT);
      usingComputedSimSignals = true;
    } else {
      lastSeedStatus = "NO_DEVICES";
      return;
    }

    lastAbsA = absA;
    lastAbsB = absB;
    lastUsingComputedSimSignals = usingComputedSimSignals;

    CrtSolution solution =
        resolveTurretFromSensors(
            absA,
            absB,
            ratioA,
            ratioB,
            SubsystemConstants.TURRET_MAX_ROTATIONS,
            SubsystemConstants.TURRET_ABS_MATCH_TOL_ROT);

    Logger.recordOutput("Turret/CRT/AbsA", absA);
    Logger.recordOutput("Turret/CRT/AbsB", absB);
    Logger.recordOutput("Turret/CRT/UsingComputedSimSignals", usingComputedSimSignals);

    if (solution == null) {
      Logger.recordOutput("Turret/CRT/SolutionFound", false);
      lastSeedStatus = "NO_SOLUTION";
      return;
    }

    double turretRotations = solution.turretRotations();
    double rotorRotations = turretRotations * SubsystemConstants.TURRET_MOTOR_TO_TURRET_RATIO;
    StatusCode seedStatus = turretMotor.setPosition(turretRotations, 0.25);
    if (!seedStatus.isOK()) {
      PhoenixUtil.tryUntilOk(4, () -> turretMotor.setPosition(turretRotations, 0.25));
      seedStatus = turretMotor.setPosition(turretRotations, 0.25);
    }

    if (seedStatus.isOK()) {
      if (Constants.currentMode == Constants.Mode.SIM) {
        var simState = turretMotor.getSimState();
        simState.setRotorVelocity(0.0);
      }
      rotorSeededFromAbs = true;
      lastSeededTurretDeg = turretRotations * 360.0;
      lastSeedError = solution.error();
      Logger.recordOutput("Turret/CRT/SolutionFound", true);
      Logger.recordOutput("Turret/CRT/SeededTurretDeg", lastSeededTurretDeg);
      Logger.recordOutput("Turret/CRT/SeededRotorRot", rotorRotations);
      Logger.recordOutput("Turret/CRT/MatchErrorRot", lastSeedError);

      // Nudge the setpoint once to the seeded position, but don't hold it forever
      turret
          .setAngle(Degrees.of(lastSeededTurretDeg))
          .ignoringDisable(true)
          .withTimeout(0.05)
          .schedule();
    }

    lastSeedStatus = seedStatus.toString();
    Logger.recordOutput("Turret/CRT/SeedStatus", seedStatus.toString());
    Logger.recordOutput("Turret/CRT/Seeded", rotorSeededFromAbs);
  }

  private void updateCancoderSimState() {
    double turretRotations = getAngle().in(Degrees) / 360.0;
    double timestamp = Timer.getFPGATimestamp();
    double turretVelocityRps = 0.0;
    if (Double.isFinite(lastSimTurretRotations) && Double.isFinite(lastSimTimestampSec)) {
      double dt = timestamp - lastSimTimestampSec;
      if (dt > 1e-6) {
        turretVelocityRps = (turretRotations - lastSimTurretRotations) / dt;
      }
    }
    lastSimTurretRotations = turretRotations;
    lastSimTimestampSec = timestamp;

    double ratioA = cancoderRatio(SubsystemConstants.TURRET_ABS_GEAR_A_TEETH);
    double ratioB = cancoderRatio(SubsystemConstants.TURRET_ABS_GEAR_B_TEETH);

    cancoderSimA.setSupplyVoltage(12.0);
    cancoderSimA.setRawPosition(ratioA * turretRotations);
    cancoderSimA.setVelocity(ratioA * turretVelocityRps);

    cancoderSimB.setSupplyVoltage(12.0);
    cancoderSimB.setRawPosition(ratioB * turretRotations);
    cancoderSimB.setVelocity(ratioB * turretVelocityRps);
  }

  // try all feasible wrap counts for encoder A, then pick the turret rotation whose
  // predicted encoder B value best matches the wrapped value
  private CrtSolution resolveTurretFromSensors(
      double absA,
      double absB,
      double ratioA,
      double ratioB,
      double maxTurretRotations,
      double matchTolerance) {
    double bestErr = Double.MAX_VALUE;
    double secondErr = Double.MAX_VALUE;
    double bestRot = Double.NaN;

    // absA is modulo-1; n is the integer wrap count, bound so turretRot stays in [0, max]
    // Expected loop count is ~= (ratioA * maxTurretRotations + 3). The lower this is, the 
    // more reasonable the search time will be. For example, if your turret gearbox is 12:50, 10:110,
    // then you can drive the encoders off the 50t spur with a 27t and 31t gear, which gives you ~ 1.52 rotations
    // (450 degrees) of absolute coverage, and a maximum of 29 loop iterations.
    int minNA = (int) Math.floor(-absA);
    int maxNA = (int) Math.ceil(ratioA * maxTurretRotations - absA + 1.0);

    for (int n = minNA; n <= maxNA; n++) {
      //"candidate" turret rotation derived from encoder A
      double turretRot = (absA + n) / ratioA;
      // avoid rejecting edge values due to rounding
      if (turretRot < -1e-6 || turretRot > maxTurretRotations + 1e-6) {
        continue;
      }
      // predict encoder B for this candiate and score by best modular error
      double predictedB = wrap01(ratioB * turretRot);
      double err = modularError(predictedB, absB);
      if (err < bestErr) {
        secondErr = bestErr;
        bestErr = err;
        bestRot = turretRot;
      } else if (err < secondErr) {
        secondErr = err;
      }
    }

    // matchTolerance is in fractional rotations of the encoder gear
    if (!Double.isFinite(bestRot) || bestErr > matchTolerance) {
      return null;
    }

    // if two candidates match nearly equally, treat them as ambiguous
    if (secondErr <= matchTolerance && Math.abs(secondErr - bestErr) < 1e-3) {
      return null;
    }

    return new CrtSolution(bestRot, bestErr);
  }

  private static double wrap01(double rotations) {
    double wrapped = rotations % 1.0;
    if (wrapped < 0.0) {
      wrapped += 1.0;
    }
    return wrapped;
  }

  private static double modularError(double a, double b) {
    double diff = Math.abs(a - b);
    return diff > 0.5 ? 1.0 - diff : diff;
  }

  private double cancoderRatio(int drivenTeeth) {
    // ratio = (turret gear/50t gear) * ( 50t gear/encoder gear)
    return SubsystemConstants.TURRET_STAGE2_RATIO
        * (SubsystemConstants.TURRET_STAGE1_GEAR_TEETH / drivenTeeth);
  }

  private void logCrtCoverage() {
    // LCM of the encoder gear teeth give the repeat period of the combined pattern
    double coverageRot =
        lcm(SubsystemConstants.TURRET_ABS_GEAR_A_TEETH, SubsystemConstants.TURRET_ABS_GEAR_B_TEETH)
            / (SubsystemConstants.TURRET_STAGE2_RATIO
                * SubsystemConstants.TURRET_STAGE1_GEAR_TEETH);
    Logger.recordOutput("Turret/CRT/UniqueCoverageRot", coverageRot);
    Logger.recordOutput(
        "Turret/CRT/CoverageSatisfiesRange",
        coverageRot >= SubsystemConstants.TURRET_MAX_ROTATIONS);

    // max amount of loop iterations in resolveTurretFromSensors
    double ratioA = cancoderRatio(SubsystemConstants.TURRET_ABS_GEAR_A_TEETH);
    double theoreticalMaxIterations =
        Math.ceil(ratioA * SubsystemConstants.Turret_MAX_ROTATIONS) + 3.0;
    Logger.recordOutput("Turret/CRT/TheoreticalMaxIterations", theoreticalMaxIterations);
  }

  private static int gcd(int a, int b) {
    int x = Math.abs(a);
    int y = Math.abs(b);
    while (y != 0) {
      int tmp = y;
      y = x % y;
      x = tmp;
    }
    return x;
  }

  private static int lcm(int a, int b) {
    return (a / gcd(a, b)) * b;
  }

  private static record CrtSolution(double turretRotations, double error) {}
}
