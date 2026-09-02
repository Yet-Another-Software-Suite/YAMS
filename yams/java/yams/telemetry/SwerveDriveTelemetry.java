package yams.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.struct.Struct;
import java.util.Map;
import yams.mechanisms.swerve.SwerveDrive;
import yams.mechanisms.swerve.SwerveModule;

public class SwerveDriveTelemetry {
  private final SwerveDriveTelemetryConfig m_config;
  private Map<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> m_doubleTelemetry;
  private Map<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> m_structTelemetry;
  private Map<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>>
      m_structArrayTelemetry;
  private Map<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> m_boolTelemetry;
  private NetworkTable m_dataNt;
  private NetworkTable m_tuningNt;

  /**
   * Create SwerveDrive telemetry for logging in NetworkTables and DataLog.
   */
  public SwerveDriveTelemetry(SwerveDriveTelemetryConfig config) {
    m_config = config;
  }

  /**
   * Setup telemetry for the drive.
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
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
        m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        dt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(dt::setupDataLog);
    }
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
        m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        stt.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(stt::setupDataLog);
    }
    for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>>
             entry : m_structArrayTelemetry.entrySet()) {
      var stat = entry.getValue();
      if (!stat.enabled) {
        continue;
      }
      if (m_config.getNT4Enabled()) {
        stat.setupNetworkTables(m_dataNt, m_tuningNt);
      }
      m_config.getDataLogName().ifPresent(stat::setupDataLog);
    }
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> entry :
        m_boolTelemetry.entrySet()) {
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
   * @param drive {@link SwerveDrive} to publish telemetry from.
   */
  @SuppressWarnings("unchecked")
  public void publish(SwerveDrive drive) {
    var cfg = drive.getConfig();
    for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
        m_structTelemetry.entrySet()) {
      var stt = entry.getValue();
      if (!stt.enabled) {
        continue;
      }
      switch (stt.getField()) {
        case Pose -> ((StructTelemetry<Pose2d, StructTelemetryField>) stt).set(drive.getPose());
        case DesiredRobotRelativeChassisSpeeds ->
          ((StructTelemetry<ChassisSpeeds, StructTelemetryField>) stt)
              .set(drive.getDesiredChassisSpeeds());
        case CurrentRobotRelativeChassisSpeeds ->
          ((StructTelemetry<ChassisSpeeds, StructTelemetryField>) stt)
              .set(drive.getRobotRelativeSpeed());
        case FieldRelativeChassisSpeeds ->
          ((StructTelemetry<ChassisSpeeds, StructTelemetryField>) stt)
              .set(drive.getFieldRelativeSpeed());
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
        m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.enabled)
        continue;
      switch (dt.getField()) {
        case Gyro -> dt.set(drive.getGyroAngle().in(Degrees));
      }
    }
    for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>>
             entry : m_structArrayTelemetry.entrySet()) {
      var stat = entry.getValue();
      if (!stat.enabled) {
        continue;
      }
      switch (stat.getField()) {
        case DesiredModuleStates ->
          ((StructArrayTelemetry<SwerveModuleState, StructArrayTelemetryField>) stat)
              .set(drive.getDesiredModuleStates());
        case CurrentModuleStates ->
          ((StructArrayTelemetry<SwerveModuleState, StructArrayTelemetryField>) stat)
              .set(drive.getModuleStates());
      }
    }
  }

  /**
   * Apply the tuning values from {@link NetworkTable} to the {@link SwerveDrive}
   * @param drive {@link SwerveDrive} to control.
   */
  @SuppressWarnings("unchecked")
  public void applyTuningValues(SwerveDrive drive) {
    var cfg = drive.getConfig();
    var translationPID = cfg.getTranslationPID();
    var rotationPID = cfg.getRotationPID();
    boolean nt4Enabled = m_config.getNT4Enabled();

    var autoAlignBt = m_boolTelemetry.get(BooleanTelemetryField.AutoAlignEnabled);
    var driveTuningBt = m_boolTelemetry.get(BooleanTelemetryField.ModulesDriveTuningEnabled);
    var azimuthTuningBt = m_boolTelemetry.get(BooleanTelemetryField.ModulesAzimuthTuningEnabled);

    boolean autoAlignOn = autoAlignBt.enabled && nt4Enabled && autoAlignBt.get();
    boolean driveTuningOn = driveTuningBt.enabled && nt4Enabled && driveTuningBt.get();
    boolean azimuthTuningOn = azimuthTuningBt.enabled && nt4Enabled && azimuthTuningBt.get();

    // Only one tuning mode may drive the chassis at a time; enforce priority order
    // (auto-align > drive tuning > azimuth tuning) and force the losers back off in
    // NetworkTables so the dashboard reflects what's actually happening.
    if (autoAlignOn) {
      if (driveTuningOn) {
        driveTuningBt.forceSet(false);
        driveTuningOn = false;
      }
      if (azimuthTuningOn) {
        azimuthTuningBt.forceSet(false);
        azimuthTuningOn = false;
      }
    } else if (driveTuningOn && azimuthTuningOn) {
      azimuthTuningBt.forceSet(false);
      azimuthTuningOn = false;
    }

    if (autoAlignOn) {
      var poseX = m_doubleTelemetry.get(DoubleTelemetryField.AutoAlignPoseX);
      var poseY = m_doubleTelemetry.get(DoubleTelemetryField.AutoAlignPoseY);
      var poseRotation = m_doubleTelemetry.get(DoubleTelemetryField.AutoAlignPoseRotation);
      var targetPose =
          new Pose2d(poseX.get(), poseY.get(), Rotation2d.fromDegrees(poseRotation.get()));
      drive.setFieldRelativeChassisSpeeds(drive.driveToPoseSetpoint(targetPose));
    }

    if (driveTuningBt.enabled && nt4Enabled) {
      var modules = drive.getModules();
      var states = new SwerveModuleState[modules.length];
      if (driveTuningOn) {
        var velocity = MetersPerSecond.of(
            m_doubleTelemetry.get(DoubleTelemetryField.ModulesDriveVelocity).get());
        for (int i = 0; i < modules.length; i++) {
          states[i] = new SwerveModuleState(velocity, modules[i].getState().angle);
        }
        drive.setSwerveModuleStates(states);
      } else if (!autoAlignOn) {
        // Drive tuning is off (and auto-align isn't driving the chassis instead); make sure the
        // modules don't keep spinning at whatever velocity was last commanded while it was on.
        for (int i = 0; i < modules.length; i++) {
          states[i] = new SwerveModuleState(MetersPerSecond.of(0), modules[i].getState().angle);
        }
        drive.setSwerveModuleStates(states);
      }
    }

    if (azimuthTuningOn) {
      var angle =
          Rotation2d.fromDegrees(m_doubleTelemetry.get(DoubleTelemetryField.ModulesAzimuthAngle).get());
      var modules = drive.getModules();
      var states = new SwerveModuleState[modules.length];
      for (int i = 0; i < modules.length; i++) {
        states[i] = new SwerveModuleState(0, angle);
      }
      drive.setSwerveModuleStates(states);
    }

    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
        m_doubleTelemetry.entrySet()) {
      var dt = entry.getValue();
      if (!dt.tunable())
        continue;
      switch (dt.getField()) {
        case TranslationP -> {
          translationPID.setP(dt.get());
          drive.setTranslationPID(translationPID);
        }
        case TranslationI -> {
          translationPID.setI(dt.get());
          drive.setTranslationPID(translationPID);
        }
        case TranslationD -> {
          translationPID.setD(dt.get());
          drive.setTranslationPID(translationPID);
        }
        case RotationP -> {
          rotationPID.setP(dt.get());
          drive.setRotationPID(rotationPID);
        }
        case RotationI -> {
          rotationPID.setI(dt.get());
          drive.setRotationPID(rotationPID);
        }
        case RotationD -> {
          rotationPID.setD(dt.get());
          drive.setRotationPID(rotationPID);
        }
        case ModulesDriveP -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKp(dt.get());
          }
        }
        case ModulesDriveI -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKi(dt.get());
          }
        }
        case ModulesDriveD -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKd(dt.get());
          }
        }
        case ModulesAzimuthP -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKp(dt.get());
          }
        }
        case ModulesAzimuthI -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKi(dt.get());
          }
        }
        case ModulesAzimuthD -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKd(dt.get());
          }
        }
        case ModulesDriveKs -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKs(dt.get());
          }
        }
        case ModulesDriveKv -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKv(dt.get());
          }
        }
        case ModulesDriveKa -> {
          for (SwerveModule module : drive.getModules()) {
            module.getDriveMotorController().setKa(dt.get());
          }
        }
        case ModulesAzimuthKs -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKs(dt.get());
          }
        }
        case ModulesAzimuthKv -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKv(dt.get());
          }
        }
        case ModulesAzimuthKa -> {
          for (SwerveModule module : drive.getModules()) {
            module.getAzimuthMotorController().setKa(dt.get());
          }
        }
        default -> {
        }
      }
    }
  }

  /**
   * Close and unpublish telemetry.
   */
  public void close() {
    if (m_doubleTelemetry != null) {
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry<DoubleTelemetryField>> entry :
          m_doubleTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_structTelemetry != null) {
      for (Map.Entry<StructTelemetryField, StructTelemetry<?, StructTelemetryField>> entry :
          m_structTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_structArrayTelemetry != null) {
      for (Map.Entry<StructArrayTelemetryField, StructArrayTelemetry<?, StructArrayTelemetryField>>
               entry : m_structArrayTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
    if (m_boolTelemetry != null) {
      for (Map.Entry<BooleanTelemetryField, BooleanTelemetry<BooleanTelemetryField>> entry :
          m_boolTelemetry.entrySet()) {
        entry.getValue().close();
      }
    }
  }

  /**
   * Struct telemetry field for {@link SwerveDrive}s.
   */
  public enum StructTelemetryField {
    /**
     * Estimated {@link Pose2d} of the robot, as reported by the {@link edu.wpi.first.math.estimator.SwerveDrivePoseEstimator}.
     */
    Pose("pose", Pose2d.struct, new Pose2d(), false),
    /**
     * Last-commanded desired robot relative {@link ChassisSpeeds}.
     */
    DesiredRobotRelativeChassisSpeeds(
        "chassis/desired", ChassisSpeeds.struct, new ChassisSpeeds(), false),
    /**
     * Measured robot relative {@link ChassisSpeeds}.
     */
    CurrentRobotRelativeChassisSpeeds(
        "chassis/current", ChassisSpeeds.struct, new ChassisSpeeds(), false),
    /**
     * Measured field relative {@link ChassisSpeeds}.
     */
    FieldRelativeChassisSpeeds("chassis/field", ChassisSpeeds.struct, new ChassisSpeeds(), false);

    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * {@link Struct} serializer for the field's value type.
     */
    private final Struct<?> struct;
    /**
     * Default value of the struct telemetry field.
     */
    private final Object defaultValue;
    /**
     * Tunable field?
     */
    private final boolean tunable;

    /**
     * Create a struct telemetry field.
     *
     * @param fieldName    Field for {@link edu.wpi.first.networktables.NetworkTable}
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

  /**
   * Struct array telemetry field for {@link SwerveDrive}s.
   */
  public enum StructArrayTelemetryField {
    /**
     * Last-commanded desired {@link SwerveModuleState}s.
     */
    DesiredModuleStates(
        "states/desired", SwerveModuleState.struct, new SwerveModuleState[0], false),
    /**
     * Measured {@link SwerveModuleState}s.
     */
    CurrentModuleStates(
        "states/current", SwerveModuleState.struct, new SwerveModuleState[0], false);

    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * {@link Struct} serializer for the field's array element type.
     */
    private final Struct<?> struct;
    /**
     * Default value of the struct array telemetry field.
     */
    private final Object[] defaultValue;
    /**
     * Tunable field?
     */
    private final boolean tunable;

    /**
     * Create a struct array telemetry field.
     *
     * @param fieldName    Field for {@link edu.wpi.first.networktables.NetworkTable}
     * @param struct       {@link Struct} serializer for the field's array element type {@link T}.
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     * @param <T>          Type of the field's array elements.
     */
    <T> StructArrayTelemetryField(
        String fieldName, Struct<T> struct, T[] defaultValue, boolean tunable) {
      key = fieldName;
      this.struct = struct;
      this.defaultValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link StructArrayTelemetry} object for non-static usage.
     *
     * @param <T> Type of the field's array elements, must match the type this field was declared
     *     with.
     * @return {@link StructArrayTelemetry}
     */
    @SuppressWarnings("unchecked")
    public <T> StructArrayTelemetry<T, StructArrayTelemetryField> create() {
      return new StructArrayTelemetry<>(key, (T[]) defaultValue, this, (Struct<T>) struct, tunable);
    }
  }

  /**
   * Double telemetry field for {@link SwerveDrive}s.
   */
  public enum DoubleTelemetryField {
    /**
     * Translational proporational gain for auto-aligning the robot.
     */
    TranslationP("autoalign/translation/p", 0, true, "meters"),
    /**
     * Translational integral gain for auto-aligning the robot.
     */
    TranslationI("autoalign/translation/i", 0, true, "meters"),
    /**
     * Translational derivative gain for auto-aligning the robot.
     */
    TranslationD("autoalign/translation/d", 0, true, "meters"),
    /**
     * Rotational proporational gain for auto-aligning the robot.
     */
    RotationP("autoalign/rotation/p", 0, true, "radians"),
    /**
     * Rotational integral gain for auto-aligning the robot.
     */
    RotationI("autoalign/rotation/i", 0, true, "radians"),
    /**
     * Rotational derivative gain for auto-aligning the robot.
     */
    RotationD("autoalign/rotation/d", 0, true, "radians"),
    /**
     * X position of the auto-align target pose, in meters.
     */
    AutoAlignPoseX("autoalign/pose/x", 3, true, "meters"),
    /**
     * Y position of the auto-align target pose, in meters.
     */
    AutoAlignPoseY("autoalign/pose/y", 3, true, "meters"),
    /**
     * Rotation of the auto-align target pose, in degrees.
     */
    AutoAlignPoseRotation("autoalign/pose/rot", 0, true, "degrees"),
    /**
     * Proportional gain for tuning every module's drive motor closed-loop controller.
     */
    ModulesDriveP("modules/drive/feedback/p", 0, true, "none"),
    /**
     * Integral gain for tuning every module's drive motor closed-loop controller.
     */
    ModulesDriveI("modules/drive/feedback/i", 0, true, "none"),
    /**
     * Derivative gain for tuning every module's drive motor closed-loop controller.
     */
    ModulesDriveD("modules/drive/feedback/d", 0, true, "none"),
    /**
     * Static feedforward gain for tuning every module's drive motor.
     */
    ModulesDriveKs("modules/drive/feedforward/s", 0, true, "none"),
    /**
     * Velocity feedforward gain for tuning every module's drive motor.
     */
    ModulesDriveKv("modules/drive/feedforward/v", 0, true, "none"),
    /**
     * Acceleration feedforward gain for tuning every module's drive motor.
     */
    ModulesDriveKa("modules/drive/feedforward/a", 0, true, "none"),
    /**
     * Tunable velocity setpoint applied to every module's drive motor, in meters per second.
     */
    ModulesDriveVelocity("modules/drive/velocity", 0, true, "meters_per_second"),
    /**
     * Proportional gain for tuning every module's azimuth motor closed-loop controller.
     */
    ModulesAzimuthP("modules/azimuth/feedback/p", 0, true, "none"),
    /**
     * Integral gain for tuning every module's azimuth motor closed-loop controller.
     */
    ModulesAzimuthI("modules/azimuth/feedback/i", 0, true, "none"),
    /**
     * Derivative gain for tuning every module's azimuth motor closed-loop controller.
     */
    ModulesAzimuthD("modules/azimuth/feedback/d", 0, true, "none"),
    /**
     * Static feedforward gain for tuning every module's azimuth motor.
     */
    ModulesAzimuthKs("modules/azimuth/feedforward/s", 0, true, "none"),
    /**
     * Velocity feedforward gain for tuning every module's azimuth motor.
     */
    ModulesAzimuthKv("modules/azimuth/feedforward/v", 0, true, "none"),
    /**
     * Acceleration feedforward gain for tuning every module's azimuth motor.
     */
    ModulesAzimuthKa("modules/azimuth/feedforward/a", 0, true, "none"),
    /**
     * Tunable angle setpoint applied to every module's azimuth motor, in degrees.
     */
    ModulesAzimuthAngle("modules/azimuth/angle", 0, true, "degrees"),
    /**
     * Gyro angle, in degrees.
     */
    Gyro("gyro", 0, false, "degrees");

    /**
     * Default value of the double telemetry field.
     */
    private final double defaultVal;
    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * Tunable field?
     */
    private final boolean tunable;
    /**
     * Unit of the telemetry field.
     */
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

  /**
   * Boolean telemetry field for {@link SwerveDrive}s.
   */
  public enum BooleanTelemetryField {
    /**
     * Enables or disables driving to the auto-align target pose.
     */
    AutoAlignEnabled("autoalign/enabled", false, true),
    /**
     * Enables or disables live tuning of every module's drive motor PID gains and velocity
     * setpoint.
     */
    ModulesDriveTuningEnabled("modules/drive/enabled", false, true),
    /**
     * Enables or disables live tuning of every module's azimuth motor PID gains and angle
     * setpoint.
     */
    ModulesAzimuthTuningEnabled("modules/azimuth/enabled", false, true);

    /**
     * Default value of the boolean telemetry field.
     */
    private final boolean defaultVal;
    /**
     * Key that the telemetry is stored at.
     */
    private final String key;
    /**
     * Tunable field?
     */
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
