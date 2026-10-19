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
    if (!m_config.getNT4Enabled()) {
      return;
    }
    var modules = drive.getModules();
    var translationPID = drive.getConfig().getTranslationPID();
    var rotationPID = drive.getConfig().getRotationPID();

    // Read which tuning modes are on, in priority order (auto-align > drive tuning > azimuth tuning),
    // the order the fields are declared in. Only one may drive the chassis at a time, so a lower one
    // that is on is forced back off in NetworkTables, so the dashboard reflects what's actually
    // happening. Auto-align fields are disabled when either drive to pose controller is missing.
    boolean autoAlignOn = false;
    boolean driveTuningOn = false;
    boolean driveInPlaceOn = false;
    boolean azimuthTuningOn = false;
    for (BooleanTelemetryField field : BooleanTelemetryField.values()) {
      var bt = m_boolTelemetry.get(field);
      if (!bt.enabled) {
        continue;
      }
      switch (field) {
        case AutoAlignEnabled:
          autoAlignOn = bt.get();
          break;
        case ModulesDriveTuningEnabled:
          driveTuningOn = bt.get();
          if (driveTuningOn && autoAlignOn) {
            bt.forceSet(false);
            driveTuningOn = false;
          }
          break;
        case ModulesDriveInPlace:
          driveInPlaceOn = bt.get();
          break;
        case ModulesDriveVoltageEnabled:
          // A voltage is only applied while its tuning mode is on.
          setVoltageControl(drive, false, driveTuningOn && bt.get());
          break;
        case ModulesAzimuthTuningEnabled:
          azimuthTuningOn = bt.get();
          if (azimuthTuningOn && (autoAlignOn || driveTuningOn)) {
            bt.forceSet(false);
            azimuthTuningOn = false;
          }
          break;
        case ModulesAzimuthVoltageEnabled:
          setVoltageControl(drive, true, azimuthTuningOn && bt.get());
          break;
      }
    }

    // Set every gain that changed, and drive the modules in the tuning mode that is on.
    for (DoubleTelemetryField field : DoubleTelemetryField.values()) {
      var dt = m_doubleTelemetry.get(field);
      if (!dt.enabled) {
        continue;
      }
      // Whether a gain changed since the last loop, so it is only set when it does.
      boolean changed = dt.tunable();
      switch (field) {
        case TranslationP:
          if (changed) {
            translationPID.ifPresent(pid -> {
              pid.setP(dt.get());
              drive.setTranslationPID(pid);
            });
          }
          break;
        case TranslationI:
          if (changed) {
            translationPID.ifPresent(pid -> {
              pid.setI(dt.get());
              drive.setTranslationPID(pid);
            });
          }
          break;
        case TranslationD:
          if (changed) {
            translationPID.ifPresent(pid -> {
              pid.setD(dt.get());
              drive.setTranslationPID(pid);
            });
          }
          break;
        case RotationP:
          if (changed) {
            rotationPID.ifPresent(pid -> {
              pid.setP(dt.get());
              drive.setRotationPID(pid);
            });
          }
          break;
        case RotationI:
          if (changed) {
            rotationPID.ifPresent(pid -> {
              pid.setI(dt.get());
              drive.setRotationPID(pid);
            });
          }
          break;
        case RotationD:
          if (changed) {
            rotationPID.ifPresent(pid -> {
              pid.setD(dt.get());
              drive.setRotationPID(pid);
            });
          }
          break;
        case AutoAlignPoseX:
        case AutoAlignPoseY:
          // Read with the rotation.
          break;
        case AutoAlignPoseRotation:
          if (autoAlignOn) {
            var targetPose = new Pose2d(m_doubleTelemetry.get(DoubleTelemetryField.AutoAlignPoseX).get(),
                                        m_doubleTelemetry.get(DoubleTelemetryField.AutoAlignPoseY).get(),
                                        Rotation2d.fromDegrees(dt.get()));
            drive.setFieldRelativeChassisSpeeds(drive.driveToPoseSetpoint(targetPose));
          }
          break;
        // Every module's drive or azimuth gain.
        case ModulesDriveP:
        case ModulesAzimuthP:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveP ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKp(dt.get());
            }
          }
          break;
        case ModulesDriveI:
        case ModulesAzimuthI:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveI ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKi(dt.get());
            }
          }
          break;
        case ModulesDriveD:
        case ModulesAzimuthD:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveD ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKd(dt.get());
            }
          }
          break;
        case ModulesDriveKs:
        case ModulesAzimuthKs:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveKs ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKs(dt.get());
            }
          }
          break;
        case ModulesDriveKv:
        case ModulesAzimuthKv:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveKv ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKv(dt.get());
            }
          }
          break;
        case ModulesDriveKa:
        case ModulesAzimuthKa:
          if (changed) {
            for (SwerveModule module : modules) {
              (field == DoubleTelemetryField.ModulesDriveKa ? module.getDriveMotorController() : module.getAzimuthMotorController()).setKa(dt.get());
            }
          }
          break;
        case ModulesDriveVelocity:
          if (driveTuningOn && !m_driveVoltageControl) {
            var states = new SwerveModuleVelocity[modules.length];
            for (int i = 0; i < modules.length; i++) {
              states[i] = new SwerveModuleVelocity(dt.get(), driveTuningAngle(modules[i], driveInPlaceOn));
            }
            drive.setSwerveModuleStates(states);
          } else if (!driveTuningOn && !autoAlignOn && !azimuthTuningOn) {
            // No tuning mode is driving the chassis; make sure the modules don't keep spinning at
            // whatever velocity was last commanded while drive tuning was on. Azimuth tuning holds
            // the wheels still itself.
            holdWheels(drive);
          }
          break;
        case ModulesDriveVoltage:
          if (m_driveVoltageControl) {
            // The wheels are not optimized, so they always drive the same way.
            for (SwerveModule module : modules) {
              module.getDriveMotorController().setVoltage(Volts.of(dt.get()));
              module.getAzimuthMotorController().setPosition(driveTuningAngle(module, driveInPlaceOn).getMeasure());
            }
          }
          break;
        case ModulesAzimuthAngle:
          if (azimuthTuningOn && !m_azimuthVoltageControl) {
            // Zero speed states would be held where the wheels are by the velocity deadband, so turn
            // the wheels without driving them.
            drive.setAzimuthAngles(Rotation2d.fromDegrees(dt.get()));
          }
          break;
        case ModulesAzimuthVoltage:
          if (m_azimuthVoltageControl) {
            // Hold the drive motors still while the azimuth motors turn.
            for (SwerveModule module : modules) {
              module.getDriveMotorController().setVelocity(MetersPerSecond.of(0).plus(module.getCouplingVelocity()));
              module.getAzimuthMotorController().setVoltage(Volts.of(dt.get()));
            }
          }
          break;
        case Gyro:
          break;
      }
    }
  }

  /**
   * The angle a module points while drive tuning: forward, or, to spin in place, tangent to its
   * position around the robot center so a positive velocity spins the robot counter-clockwise
   * (WPILib's positive rotation direction).
   *
   * @param module  The module.
   * @param inPlace True to spin in place.
   * @return The angle.
   */
  private static Rotation2d driveTuningAngle(SwerveModule module, boolean inPlace) {
    if (!inPlace) {
      return Rotation2d.ZERO;
    }
    var moduleLocation = module.getConfig().getLocation().orElseThrow();
    return moduleLocation.getAngle().orElse(new Rotation2d()).plus(Rotation2d.fromDegrees(90));
  }

  /**
   * Stop applying a tuning voltage, as when the tuning command ends, and hold every wheel still where
   * it is with its closed loop controller.
   *
   * @param drive {@link SwerveDrive} being tuned.
   */
  public void stopTuning(SwerveDrive drive) {
    setVoltageControl(drive, false, false);
    setVoltageControl(drive, true, false);
  }

  /**
   * Switch the drive or azimuth motors between a tuning voltage and their closed loop controllers. A
   * closed loop controller run on the SystemCore would replace the voltage on its next loop, so it is
   * stopped while the voltage is applied.
   *
   * @param drive   {@link SwerveDrive} being tuned.
   * @param azimuth True for the azimuth motors, false for the drive motors.
   * @param on      True to apply a voltage, false to go back to the closed loop controller.
   */
  private void setVoltageControl(SwerveDrive drive, boolean azimuth, boolean on) {
    if (on == (azimuth ? m_azimuthVoltageControl : m_driveVoltageControl)) {
      return;
    }
    if (azimuth) {
      m_azimuthVoltageControl = on;
    } else {
      m_driveVoltageControl = on;
    }
    for (SwerveModule module : drive.getModules()) {
      var smc = azimuth ? module.getAzimuthMotorController() : module.getDriveMotorController();
      if (on) {
        smc.stopClosedLoopController();
      } else {
        smc.setVoltage(Volts.zero());
        smc.startClosedLoopController();
      }
    }
    if (!on) {
      // Restarted, the closed loop controllers would go back to the setpoints from before the
      // voltage, so hold every wheel where it is instead.
      holdWheels(drive);
    }
  }

  /**
   * Hold every wheel still where it is, with its closed loop controllers.
   *
   * @param drive {@link SwerveDrive} being tuned.
   */
  private static void holdWheels(SwerveDrive drive) {
    var modules = drive.getModules();
    var states = new SwerveModuleVelocity[modules.length];
    for (int i = 0; i < modules.length; i++) {
      states[i] = new SwerveModuleVelocity(MetersPerSecond.of(0), modules[i].getState().angle);
    }
    drive.setSwerveModuleStates(states);
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
