// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.telemetry;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.Volts;

import java.util.Map;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.util.struct.Struct;
import yams.core.mechanisms.swerve.SwerveDrive;
import yams.core.mechanisms.swerve.SwerveModule;

/** Publishes {@link SwerveDrive} telemetry to NetworkTables and DataLog, and applies live tuning values. */
public class SwerveDriveTelemetry {
  private final SwerveDriveTelemetryConfig                                                   m_config;
  private Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>>                   m_doubleTelemetry;
  private Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>>                m_structTelemetry;
  private Map<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> m_structArrayTelemetry;
  private Map<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>>                m_boolTelemetry;
  private NetworkTable                                                                       m_dataNt;
  private NetworkTable                                                                       m_tuningNt;
  /** Whether the drive motors are driven at a tuning voltage, without their closed loop controllers. */
  private boolean                                                                            m_driveVoltageControl;
  /** Whether the azimuth motors are driven at a tuning voltage, without their closed loop controllers. */
  private boolean                                                                            m_azimuthVoltageControl;

  /**
   * Create SwerveDrive telemetry for logging in NetworkTables and DataLog.
   *
   * @param config {@link SwerveDriveTelemetryConfig} selecting what to publish.
   */
  public SwerveDriveTelemetry(SwerveDriveTelemetryConfig config) {
    m_config = config;
  }

  /**
   * Setup telemetry for the drive.
   *
   * @param drive {@link SwerveDrive} to use for telemetry.
   */
  public void setupTelemetry(SwerveDrive drive) {
    var driveName = drive.getName();
    for (SwerveModule module : drive.getModules()) {
      module.setupTelemetry(driveName);
    }
    m_dataNt = NetworkTableInstance.getDefault().getTable("Mechanisms").getSubTable(driveName);
    m_tuningNt = NetworkTableInstance.getDefault().getTable("Tuning").getSubTable(driveName);
    m_doubleTelemetry = m_config.getDoubleFields(drive);
    m_structTelemetry = m_config.getStructFields();
    m_structArrayTelemetry = m_config.getStructArrayFields();
    m_boolTelemetry = m_config.getBoolFields();
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry : m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        dt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(dt::setupDataLog);
    }
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry : m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        stt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(stt::setupDataLog);
    }
    for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> entry : m_structArrayTelemetry.entrySet()) {
      var stat = entry.getValue();
      if (!stat.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        stat.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(stat::setupDataLog);
    }
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> entry : m_boolTelemetry.entrySet()) {
      var bt = entry.getValue();
      if (!bt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        bt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(bt::setupDataLog);
    }
  }

  /**
   * Publish telemetry to NetworkTables and DataLog
   *
   * @param drive {@link SwerveDrive} to publish telemetry from.
   */
  @SuppressWarnings("unchecked")
  public void publish(SwerveDrive drive) {
    var cfg = drive.getConfig();
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry : m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      switch (stt.getField()) {
        case Pose -> ((StructTelemetry<Pose2d, StructTelemetryField>) stt).set(drive.getPose());
        case DesiredRobotRelativeChassisSpeeds -> ((StructTelemetry<ChassisVelocities, StructTelemetryField>) stt).set(drive.getDesiredChassisSpeeds());
        case CurrentRobotRelativeChassisSpeeds -> ((StructTelemetry<ChassisVelocities, StructTelemetryField>) stt).set(drive.getRobotRelativeSpeed());
        case FieldRelativeChassisSpeeds -> ((StructTelemetry<ChassisVelocities, StructTelemetryField>) stt).set(drive.getFieldRelativeSpeed());
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry : m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled)
        continue;
      switch (dt.getField()) {
        case Gyro -> dt.set(drive.getGyroRotation3d().getMeasureZ().in(Degrees));
      }
    }
    for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> entry : m_structArrayTelemetry.entrySet()) {
      var stat = entry.getValue();
      if (!stat.enabled) {
        continue;
      }
      switch (stat.getField()) {
        case DesiredModuleStates -> ((StructArrayTelemetry<SwerveModuleVelocity, StructArrayTelemetryField>) stat).set(drive.getDesiredModuleStates());
        case CurrentModuleStates -> ((StructArrayTelemetry<SwerveModuleVelocity, StructArrayTelemetryField>) stat).set(drive.getModuleStates());
      }
    }
  }

  /**
   * Apply the tuning values from {@link NetworkTable} to the {@link SwerveDrive}
   *
   * @param drive {@link SwerveDrive} to control.
   * @throws IllegalStateException if auto-align is enabled and a gyro angular velocity scale factor
   *                               is configured with
   *                               {@link yams.core.mechanisms.config.SwerveDriveConfig#withGyroAngularVelocityScaleFactor(double)}
   *                               but no gyro supplier was set with
   *                               {@link yams.core.mechanisms.config.SwerveDriveConfig#withGyro(java.util.function.Supplier)}.
   */
  public void applyTuningValues(SwerveDrive drive) {
    var modules = drive.getModules();

    // Read which tuning modes are on. Auto-align fields are disabled when either drive to pose
    // controller is missing.
    boolean autoAlignOn = false;
    boolean driveTuningOn = false;
    boolean driveInPlaceOn = false;
    boolean driveVoltageOn = false;
    boolean azimuthTuningOn = false;
    boolean azimuthVoltageOn = false;
    for (BooleanTelemetry<BooleanTelemetryField> bt : m_boolTelemetry.values()) {
      if (!bt.enabled || !m_config.getNT4Enabled()) {
        continue;
      }
      switch (bt.getField()) {
        case AutoAlignEnabled -> autoAlignOn = bt.get();
        case ModulesDriveTuningEnabled -> driveTuningOn = bt.get();
        case ModulesDriveInPlace -> driveInPlaceOn = bt.get();
        case ModulesDriveVoltageEnabled -> driveVoltageOn = bt.get();
        case ModulesAzimuthTuningEnabled -> azimuthTuningOn = bt.get();
        case ModulesAzimuthVoltageEnabled -> azimuthVoltageOn = bt.get();
      }
    }

    // Only one tuning mode may drive the chassis at a time; enforce priority order
    // (auto-align > drive tuning > azimuth tuning) and force the losers back off in
    // NetworkTables so the dashboard reflects what's actually happening.
    if (autoAlignOn) {
      if (driveTuningOn) {
        m_boolTelemetry.get(BooleanTelemetryField.ModulesDriveTuningEnabled).forceSet(false);
        driveTuningOn = false;
      }
      if (azimuthTuningOn) {
        m_boolTelemetry.get(BooleanTelemetryField.ModulesAzimuthTuningEnabled).forceSet(false);
        azimuthTuningOn = false;
      }
    } else if (driveTuningOn && azimuthTuningOn) {
      m_boolTelemetry.get(BooleanTelemetryField.ModulesAzimuthTuningEnabled).forceSet(false);
      azimuthTuningOn = false;
    }

    // Read the setpoints, and set every gain that changed.
    double autoAlignX = 0;
    double autoAlignY = 0;
    double autoAlignRotation = 0;
    double driveVelocity = 0;
    double driveVoltage = 0;
    double azimuthAngle = 0;
    double azimuthVoltage = 0;
    for (DoubleTelemetry<DoubleTelemetryField> dt : m_doubleTelemetry.values()) {
      if (!dt.enabled || !m_config.getNT4Enabled()) {
        continue;
      }
      switch (dt.getField()) {
        case AutoAlignPoseX -> autoAlignX = dt.get();
        case AutoAlignPoseY -> autoAlignY = dt.get();
        case AutoAlignPoseRotation -> autoAlignRotation = dt.get();
        case ModulesDriveVelocity -> driveVelocity = dt.get();
        case ModulesDriveVoltage -> driveVoltage = dt.get();
        case ModulesAzimuthAngle -> azimuthAngle = dt.get();
        case ModulesAzimuthVoltage -> azimuthVoltage = dt.get();
        default -> {
          if (dt.tunable()) {
            applyGain(drive, dt.getField(), dt.get());
          }
        }
      }
    }

    // A voltage is only applied while its tuning mode is on.
    boolean driveVoltageControl = driveTuningOn && driveVoltageOn;
    boolean azimuthVoltageControl = azimuthTuningOn && azimuthVoltageOn;
    setDriveVoltageControl(modules, driveVoltageControl);
    setAzimuthVoltageControl(modules, azimuthVoltageControl);

    if (autoAlignOn) {
      var targetPose = new Pose2d(autoAlignX, autoAlignY, Rotation2d.fromDegrees(autoAlignRotation));
      drive.setFieldRelativeChassisSpeeds(drive.driveToPoseSetpoint(targetPose));
    }

    if (m_boolTelemetry.get(BooleanTelemetryField.ModulesDriveTuningEnabled).enabled && m_config.getNT4Enabled()) {
      var states = new SwerveModuleVelocity[modules.length];
      if (driveTuningOn) {
        for (int i = 0; i < modules.length; i++) {
          var angle = Rotation2d.ZERO;
          if (driveInPlaceOn) {
            // Point each module tangent to its position around the robot center so a positive
            // velocity spins the robot counter-clockwise (WPILib's positive rotation direction).
            var moduleLocation = modules[i].getConfig().getLocation().orElseThrow();
            angle = moduleLocation.getAngle().orElse(new Rotation2d()).plus(Rotation2d.fromDegrees(90));
          }
          if (driveVoltageControl) {
            modules[i].setDriveVoltage(Volts.of(driveVoltage), angle);
          } else {
            states[i] = new SwerveModuleVelocity(MetersPerSecond.of(driveVelocity), angle);
          }
        }
        if (!driveVoltageControl) {
          drive.setSwerveModuleStates(states);
        }
      } else if (!autoAlignOn && !azimuthTuningOn) {
        // Drive tuning is off (and auto-align isn't driving the chassis instead); make sure the
        // modules don't keep spinning at whatever velocity was last commanded while it was on.
        // Azimuth tuning holds the wheels still itself.
        for (int i = 0; i < modules.length; i++) {
          states[i] = new SwerveModuleVelocity(MetersPerSecond.of(0), modules[i].getState().angle);
        }
        drive.setSwerveModuleStates(states);
      }
    }

    if (azimuthTuningOn) {
      if (azimuthVoltageControl) {
        for (SwerveModule module : modules) {
          module.setAzimuthVoltage(Volts.of(azimuthVoltage));
        }
      } else {
        // Zero speed states would be held where the wheels are by the velocity deadband, so turn
        // the wheels without driving them.
        drive.setAzimuthAngles(Rotation2d.fromDegrees(azimuthAngle));
      }
    }
  }

  /**
   * Set a gain from the tuning table: a drive to pose controller's, or every module's.
   *
   * @param drive {@link SwerveDrive} being tuned.
   * @param field The gain's field.
   * @param value The gain.
   */
  private void applyGain(SwerveDrive drive, DoubleTelemetryField field, double value) {
    var translationPID = drive.getConfig().getTranslationPID();
    var rotationPID = drive.getConfig().getRotationPID();
    switch (field) {
      case TranslationP -> translationPID.ifPresent(pid -> {
        pid.setP(value);
        drive.setTranslationPID(pid);
      });
      case TranslationI -> translationPID.ifPresent(pid -> {
        pid.setI(value);
        drive.setTranslationPID(pid);
      });
      case TranslationD -> translationPID.ifPresent(pid -> {
        pid.setD(value);
        drive.setTranslationPID(pid);
      });
      case RotationP -> rotationPID.ifPresent(pid -> {
        pid.setP(value);
        drive.setRotationPID(pid);
      });
      case RotationI -> rotationPID.ifPresent(pid -> {
        pid.setI(value);
        drive.setRotationPID(pid);
      });
      case RotationD -> rotationPID.ifPresent(pid -> {
        pid.setD(value);
        drive.setRotationPID(pid);
      });
      // The module gains.
      default -> {
        for (SwerveModule module : drive.getModules()) {
          module.applyTuningValue(field, value);
        }
      }
    }
  }

  /**
   * Stop applying a tuning voltage, as when the tuning command ends, and hold every wheel still where
   * it is with its closed loop controller.
   *
   * @param drive {@link SwerveDrive} being tuned.
   */
  public void stopTuning(SwerveDrive drive) {
    setDriveVoltageControl(drive.getModules(), false);
    setAzimuthVoltageControl(drive.getModules(), false);
  }

  /**
   * Switch the drive motors between a tuning voltage and their closed loop controllers. A closed loop
   * controller run on the SystemCore would replace the voltage on its next loop, so it is stopped
   * while the voltage is applied.
   *
   * @param modules The modules.
   * @param on      True to apply a voltage, false to go back to the closed loop controller.
   */
  private void setDriveVoltageControl(SwerveModule[] modules, boolean on) {
    if (on == m_driveVoltageControl) {
      return;
    }
    m_driveVoltageControl = on;
    for (SwerveModule module : modules) {
      var smc = module.getDriveMotorController();
      if (on) {
        smc.stopClosedLoopController();
      } else {
        smc.setVoltage(Volts.zero());
        smc.startClosedLoopController();
        // Restarted, the closed loop controller would go back to the setpoint from before the voltage.
        module.setAzimuthAngle(module.getState().angle);
      }
    }
  }

  /**
   * Switch the azimuth motors between a tuning voltage and their closed loop controllers. A closed
   * loop controller run on the SystemCore would replace the voltage on its next loop, so it is
   * stopped while the voltage is applied.
   *
   * @param modules The modules.
   * @param on      True to apply a voltage, false to go back to the closed loop controller.
   */
  private void setAzimuthVoltageControl(SwerveModule[] modules, boolean on) {
    if (on == m_azimuthVoltageControl) {
      return;
    }
    m_azimuthVoltageControl = on;
    for (SwerveModule module : modules) {
      var smc = module.getAzimuthMotorController();
      if (on) {
        smc.stopClosedLoopController();
      } else {
        smc.setVoltage(Volts.zero());
        smc.startClosedLoopController();
        // Restarted, the closed loop controller would go back to the angle from before the voltage.
        module.setAzimuthAngle(module.getState().angle);
      }
    }
  }

  /** Close and unpublish telemetry. */
  public void close() {
    if (m_doubleTelemetry != null) {
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry : m_doubleTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_structTelemetry != null) {
      for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry : m_structTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_structArrayTelemetry != null) {
      for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>> entry : m_structArrayTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_boolTelemetry != null) {
      for (Map.Entry<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> entry : m_boolTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
  }

  /** Struct telemetry field for {@link SwerveDrive}s. */
  public enum StructTelemetryField {
    /**
     * Target {@link Pose2d} for the robot driven using
     * {@link yams.commands2.swerve.SwerveDrive#driveToPose(Pose2d)}.
     */
    TargetPose("tuning/driveToPose", Pose2d.struct, new Pose2d(), true),
    /**
     * Estimated {@link Pose2d} of the robot, as reported by the
     * {@link org.wpilib.math.estimator.SwerveDrivePoseEstimator}.
     */
    Pose("pose", Pose2d.struct, new Pose2d(), false),
    /** Last-commanded desired robot relative {@link ChassisVelocities}. */
    DesiredRobotRelativeChassisSpeeds("chassis/desired", ChassisVelocities.struct, new ChassisVelocities(), false),
    /** Measured robot relative {@link ChassisVelocities}. */
    CurrentRobotRelativeChassisSpeeds("chassis/current", ChassisVelocities.struct, new ChassisVelocities(), false),
    /** Measured field relative {@link ChassisVelocities}. */
    FieldRelativeChassisSpeeds("chassis/field", ChassisVelocities.struct, new ChassisVelocities(), false);

    /** Key that the telemetry is stored at. */
    private final String key;

    /** {@link Struct} serializer for the field's value type. */
    private final Struct<?> struct;

    /** Default value of the struct telemetry field. */
    private final Object defaultValue;

    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a struct telemetry field.
     *
     * @param fieldName    Field for {@link org.wpilib.networktables.NetworkTable}
     * @param struct       {@link Struct} serializer for the field's value type {@link T}.
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     * @param <T>          Type of the field's value.
     */
    <T> StructTelemetryField(String fieldName, Struct<T> struct, T defaultValue, boolean tunable) {
      key = fieldName;
      this.struct = struct;
      this.defaultValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StructTelemetry} object for non-static usage.
     *
     * @param <T> Type of the field's value, must match the type this field was declared with.
     * @return {@link StructTelemetry}
     */
    @SuppressWarnings("unchecked")
    public <T> StructTelemetry<T, StructTelemetryField> create() {
      return new StructTelemetry<>(key, (T) defaultValue, this, (Struct<T>) struct, tunable);
    }
  }


  /** Struct array telemetry field for {@link SwerveDrive}s. */
  public enum StructArrayTelemetryField {
    /** Last-commanded desired {@link SwerveModuleVelocity}s. */
    DesiredModuleStates("states/desired", SwerveModuleVelocity.struct, new SwerveModuleVelocity[0], false),
    /** Measured {@link SwerveModuleVelocity}s. */
    CurrentModuleStates("states/current", SwerveModuleVelocity.struct, new SwerveModuleVelocity[0], false);

    /** Key that the telemetry is stored at. */
    private final String key;

    /** {@link Struct} serializer for the field's array element type. */
    private final Struct<?> struct;

    /** Default value of the struct array telemetry field. */
    private final Object[] defaultValue;

    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a struct array telemetry field.
     *
     * @param fieldName    Field for {@link org.wpilib.networktables.NetworkTable}
     * @param struct       {@link Struct} serializer for the field's array element type {@link T}.
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     * @param <T>          Type of the field's array elements.
     */
    <T> StructArrayTelemetryField(String fieldName, Struct<T> struct, T[] defaultValue, boolean tunable) {
      key = fieldName;
      this.struct = struct;
      this.defaultValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StructArrayTelemetry} object for non-static usage.
     *
     * @param <T> Type of the field's array elements, must match the type this field was declared
     *            with.
     * @return {@link StructArrayTelemetry}
     */
    @SuppressWarnings("unchecked")
    public <T> StructArrayTelemetry<T, StructArrayTelemetryField> create() {
      return new StructArrayTelemetry<>(key, (T[]) defaultValue, this, (Struct<T>) struct, tunable);
    }
  }


  /** Double telemetry field for {@link SwerveDrive}s. */
  public enum DoubleTelemetryField {
    /** Translational proporational gain for auto-aligning the robot. */
    TranslationP("autoalign/translation/p", 0, true, "meters"),
    /** Translational integral gain for auto-aligning the robot. */
    TranslationI("autoalign/translation/i", 0, true, "meters"),
    /** Translational derivative gain for auto-aligning the robot. */
    TranslationD("autoalign/translation/d", 0, true, "meters"),
    /** Rotational proporational gain for auto-aligning the robot. */
    RotationP("autoalign/rotation/p", 0, true, "radians"),
    /** Rotational integral gain for auto-aligning the robot. */
    RotationI("autoalign/rotation/i", 0, true, "radians"),
    /** Rotational derivative gain for auto-aligning the robot. */
    RotationD("autoalign/rotation/d", 0, true, "radians"),
    /** X position of the auto-align target pose, in meters. */
    AutoAlignPoseX("autoalign/setpoint/x", 3, true, "meters"),
    /** Y position of the auto-align target pose, in meters. */
    AutoAlignPoseY("autoalign/setpoint/y", 3, true, "meters"),
    /** Rotation of the auto-align target pose, in degrees. */
    AutoAlignPoseRotation("autoalign/setpoint/rot", 0, true, "degrees"),
    /** Proportional gain for tuning every module's drive motor closed-loop controller. */
    ModulesDriveP("modules/drive/feedback/p", 0, true, "none"),
    /** Integral gain for tuning every module's drive motor closed-loop controller. */
    ModulesDriveI("modules/drive/feedback/i", 0, true, "none"),
    /** Derivative gain for tuning every module's drive motor closed-loop controller. */
    ModulesDriveD("modules/drive/feedback/d", 0, true, "none"),
    /** Static feedforward gain for tuning every module's drive motor. */
    ModulesDriveKs("modules/drive/feedforward/s", 0, true, "none"),
    /** Velocity feedforward gain for tuning every module's drive motor. */
    ModulesDriveKv("modules/drive/feedforward/v", 0, true, "none"),
    /** Acceleration feedforward gain for tuning every module's drive motor. */
    ModulesDriveKa("modules/drive/feedforward/a", 0, true, "none"),
    /** Tunable velocity setpoint applied to every module's drive motor, in meters per second. */
    ModulesDriveVelocity("modules/drive/velocity", 0, true, "meters_per_second"),
    /**
     * Tunable voltage applied to every module's drive motor, without its closed loop controller, while
     * {@link BooleanTelemetryField#ModulesDriveVoltageEnabled} is on.
     */
    ModulesDriveVoltage("modules/drive/voltage/setpoint", 0, true, "volts"),
    /** Proportional gain for tuning every module's azimuth motor closed-loop controller. */
    ModulesAzimuthP("modules/azimuth/feedback/p", 0, true, "none"),
    /** Integral gain for tuning every module's azimuth motor closed-loop controller. */
    ModulesAzimuthI("modules/azimuth/feedback/i", 0, true, "none"),
    /** Derivative gain for tuning every module's azimuth motor closed-loop controller. */
    ModulesAzimuthD("modules/azimuth/feedback/d", 0, true, "none"),
    /** Static feedforward gain for tuning every module's azimuth motor. */
    ModulesAzimuthKs("modules/azimuth/feedforward/s", 0, true, "none"),
    /** Velocity feedforward gain for tuning every module's azimuth motor. */
    ModulesAzimuthKv("modules/azimuth/feedforward/v", 0, true, "none"),
    /** Acceleration feedforward gain for tuning every module's azimuth motor. */
    ModulesAzimuthKa("modules/azimuth/feedforward/a", 0, true, "none"),
    /** Tunable angle setpoint applied to every module's azimuth motor, in degrees. */
    ModulesAzimuthAngle("modules/azimuth/angle", 0, true, "degrees"),
    /**
     * Tunable voltage applied to every module's azimuth motor, without its closed loop controller,
     * while {@link BooleanTelemetryField#ModulesAzimuthVoltageEnabled} is on.
     */
    ModulesAzimuthVoltage("modules/azimuth/voltage/setpoint", 0, true, "volts"),
    /** Gyro angle, in degrees. */
    Gyro("gyro", 0, false, "degrees");

    /** Default value of the double telemetry field. */
    private final double defaultVal;

    /** Key that the telemetry is stored at. */
    private final String key;

    /** Tunable field? */
    private final boolean tunable;

    /** Unit of the telemetry field. */
    private final String unit;

    /**
     * Create a double telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     * @param unit         Unit of the telemetry field.
     */
    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable, String unit) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
      this.unit = unit;
    }

    /**
     * Create a {@link DoubleTelemetry} object for non-static usage.
     *
     * @return {@link DoubleTelemetry}
     */
    public DoubleTelemetry<DoubleTelemetryField> create() {
      return new DoubleTelemetry<>(key, defaultVal, this, tunable, unit);
    }
  }


  /** Boolean telemetry field for {@link SwerveDrive}s. */
  public enum BooleanTelemetryField {
    /** Enables or disables driving to the auto-align target pose. */
    AutoAlignEnabled("autoalign/enabled", false, true),
    /**
     * Enables or disables live tuning of every module's drive motor PID gains and velocity
     * setpoint.
     */
    ModulesDriveTuningEnabled("modules/drive/enabled", false, true),
    /**
     * When enabled ensure drive testing is done with all modules are oriented to the swerve drive
     * will drive counter clockwise positive.
     */
    ModulesDriveInPlace("modules/drive/inplace", false, true),
    /**
     * While drive tuning is on, drive every module's drive motor at
     * {@link DoubleTelemetryField#ModulesDriveVoltage} instead of the velocity setpoint, without its
     * closed loop controller, as for characterizing it.
     */
    ModulesDriveVoltageEnabled("modules/drive/voltage/enabled", false, true),
    /**
     * Enables or disables live tuning of every module's azimuth motor PID gains and angle setpoint.
     */
    ModulesAzimuthTuningEnabled("modules/azimuth/enabled", false, true),
    /**
     * While azimuth tuning is on, turn every module's azimuth motor at
     * {@link DoubleTelemetryField#ModulesAzimuthVoltage} instead of the angle setpoint, without its
     * closed loop controller, as for characterizing it.
     */
    ModulesAzimuthVoltageEnabled("modules/azimuth/voltage/enabled", false, true);

    /** Default value of the boolean telemetry field. */
    private final boolean defaultVal;

    /** Key that the telemetry is stored at. */
    private final String key;

    /** Tunable field? */
    private final boolean tunable;

    /**
     * Create a boolean telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable) {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
     *
     * @return {@link BooleanTelemetry}
     */
    public BooleanTelemetry<BooleanTelemetryField> create() {
      return new BooleanTelemetry<>(key, defaultVal, this, tunable);
    }
  }
}
