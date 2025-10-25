package yams.telemetry;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.networktables.NetworkTable;
import java.util.Map;
import yams.exceptions.SmartMotorControllerConfigurationException;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

/**
 * Smart motor controller telemetry.
 */
public class SmartMotorControllerTelemetry
{


  /**
   * {@link TelemetryVerbosity} for the {@link SmartMotorController}
   */
  private final TelemetryVerbosity                           verbosity = TelemetryVerbosity.HIGH;
  /**
   * Double telemetry fields that will be outputted.
   */
  private       Map<DoubleTelemetryField, DoubleTelemetry>   doubleFields;
  /**
   * Boolean telemetry fields that will be outputted.
   */
  private       Map<BooleanTelemetryField, BooleanTelemetry> boolFields;

  /**
   * Network table to publish to.
   */
  private NetworkTable                        dataNetworkTable;
  /**
   * Network table for tuning.
   */
  private NetworkTable                        tuningNetworkTable;
  /**
   * Telemetry config
   */
  private SmartMotorControllerTelemetryConfig config;

  /**
   * Setup Telemetry Pub/Sub fields.
   *
   * @param smartMotorController {@link SmartMotorController} used to determine what telemetry features can and should
   *                             be disabled bc they are not available.
   * @param publishTable         {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable          {@link NetworkTable} that holds all of the tuning fields.
   * @param config               {@link SmartMotorControllerTelemetryConfig} to apply.
   */
  public void setupTelemetry(SmartMotorController smartMotorController, NetworkTable publishTable,
                             NetworkTable tuningTable,
                             SmartMotorControllerTelemetryConfig config)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      dataNetworkTable = publishTable;
      tuningNetworkTable = tuningTable;
      SmartMotorControllerConfig smcConfig = smartMotorController.getConfig();
      this.config = config;
      doubleFields = config.getDoubleFields(smartMotorController);
      boolFields = config.getBoolFields(smartMotorController);
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().transformUnit(smcConfig).setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().transformUnit(smcConfig).setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }

    }
  }


  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   *
   * @param smc Smart motor controller to publish telemetry for.
   */
  public void publish(SmartMotorController smc)
  {
    SmartMotorControllerConfig cfg = smc.getConfig();
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      if (!bt.enabled)
      {
        continue;
      }
      switch (bt.getField())
      {
        case MechanismUpperLimit ->
        {
          cfg.getMechanismUpperLimit().ifPresent(upperLimit -> bt.set(smc.getMechanismPosition().gte(upperLimit)));
        }
        case MechanismLowerLimit ->
        {
          cfg.getMechanismLowerLimit().ifPresent(lowerLimit -> bt.set(smc.getMechanismPosition().lte(lowerLimit)));
        }
        case TemperatureLimit ->
        {
          cfg.getTemperatureCutoff().ifPresent(temperatureCutoff -> bt.set(smc.getTemperature()
                                                                              .gte(temperatureCutoff)));
        }
        case VelocityControl ->
        {
          bt.set(smc.getMechanismSetpointVelocity().isPresent());
        }
        case ElevatorFeedForward ->
        {
          bt.set(cfg.getElevatorFeedforward().isPresent());
        }
        case ArmFeedForward ->
        {
          bt.set(cfg.getArmFeedforward().isPresent());
        }
        case SimpleMotorFeedForward ->
        {
          bt.set(cfg.getSimpleFeedforward().isPresent());
        }
        case MotionProfile ->
        {
          bt.set(cfg.getClosedLoopController().isPresent());
        }
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (!dt.enabled)
      {
        continue;
      }
      switch (dt.getField())
      {
        case SetpointPosition ->
        {
          smc.getMechanismPositionSetpoint().ifPresent(mechSetpoint -> dt.set(
              cfg.getMechanismCircumference().isPresent() ? cfg.convertFromMechanism(mechSetpoint).in(Meters)
                                                          : mechSetpoint.in(Rotations)));
          break;
        }
        case SetpointVelocity ->
        {
          smc.getMechanismSetpointVelocity().ifPresent(mechSetpoint ->
                                                           dt.set(cfg.getMechanismCircumference().isPresent() ?
                                                                  cfg.convertFromMechanism(mechSetpoint)
                                                                     .in(MetersPerSecond) :
                                                                  mechSetpoint.in(RotationsPerSecond)));
        }
        case OutputVoltage ->
        {
          dt.set(smc.getVoltage().in(Volts));
        }
        case StatorCurrent ->
        {
          dt.set(smc.getStatorCurrent().in(Amps));
        }
        case SupplyCurrent ->
        {
          smc.getSupplyCurrent().ifPresent(current -> dt.set(current.in(Amps)));
        }
        case MotorTemperature ->
        {
          dt.set(smc.getTemperature().in(Fahrenheit));
        }
        case MeasurementPosition ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementPosition().in(Meters)));
        }
        case MeasurementVelocity ->
        {
          cfg.getMechanismCircumference().ifPresent(circumference -> dt.set(smc.getMeasurementVelocity()
                                                                               .in(MetersPerSecond)));
        }
        case MechanismPosition ->
        {
          dt.set(smc.getMechanismPosition().in(Rotations));
        }
        case MechanismVelocity ->
        {
          dt.set(smc.getMechanismVelocity().in(RotationsPerSecond));
        }
        case RotorPosition ->
        {
          dt.set(smc.getRotorPosition().in(Rotations));
        }
        case RotorVelocity ->
        {
          dt.set(smc.getRotorVelocity().in(RotationsPerSecond));
        }
      }
    }
  }


  /**
   * Apply the tuning values from {@link NetworkTable} to the {@link SmartMotorController}
   *
   * @param smartMotorController {@link SmartMotorController} to control.
   */
  public void applyTuningValues(SmartMotorController smartMotorController)
  {
    SmartMotorControllerConfig cfg = smartMotorController.getConfig();
    if (cfg.getMotorControllerMode() != SmartMotorControllerConfig.ControlMode.CLOSED_LOOP)
    {
      throw new SmartMotorControllerConfigurationException("Live tuning does not work in OPEN_LOOP",
                                                           "Cannot apply setpoints for Live Tuning.",
                                                           ".withControlMode(ControlMode.CLOSED_LOOP) instead of .withControlMode(ControlMode.OPEN_LOOP)");
    }
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      if (bt.tunable())
      {
        switch (bt.getField())
        {
          case MotorInversion -> smartMotorController.setMotorInverted(bt.get());
          case EncoderInversion -> smartMotorController.setEncoderInverted(bt.get());
        }
      }
    }
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (dt.tunable())
      {
        switch (dt.getField())
        {
          case TunableSetpointPosition ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(
                circumference -> {
                  smartMotorController.setPosition(Meters.of(dt.get()));
                }, () -> {
                  smartMotorController.setPosition(Degrees.of(dt.get()));
                });
            break;
          }
          case TunableSetpointVelocity ->
          {
            if (dt.get() == 0 && smartMotorController.getMechanismSetpointVelocity().isEmpty())
            {
              continue;
            }
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setVelocity(
                                                                MetersPerSecond.of(
                                                                    dt.get())),
                                                            () -> smartMotorController.setVelocity(
                                                                dt.get() == 0 ? null : DegreesPerSecond.of(dt.get())));
            break;
          }
          case kP -> smartMotorController.setKp(dt.get());
          case kI -> smartMotorController.setKi(dt.get());
          case kD -> smartMotorController.setKd(dt.get());
          case kS -> smartMotorController.setKs(dt.get());
          case kV -> smartMotorController.setKv(dt.get());
          case kA -> smartMotorController.setKa(dt.get());
          case kG -> smartMotorController.setKg(dt.get());
          case ClosedloopRampRate -> smartMotorController.setClosedLoopRampRate(Seconds.of(dt.get()));
          case OpenloopRampRate -> smartMotorController.setOpenLoopRampRate(Seconds.of(dt.get()));
          case SupplyCurrentLimit -> smartMotorController.setSupplyCurrentLimit(Amps.of(dt.get()));
          case StatorCurrentLimit -> smartMotorController.setStatorCurrentLimit(Amps.of(dt.get()));
          case MeasurementUpperLimit -> smartMotorController.setMeasurementUpperLimit(Meters.of(dt.get()));
          case MeasurementLowerLimit -> smartMotorController.setMeasurementLowerLimit(Meters.of(dt.get()));
          case MechanismUpperLimit -> smartMotorController.setMechanismUpperLimit(Rotations.of(dt.get()));
          case MechanismLowerLimit -> smartMotorController.setMechanismLowerLimit(Rotations.of(dt.get()));
          case MotionProfileMaxAcceleration ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(distance -> {
                                                              smartMotorController.setMotionProfileMaxAcceleration(MetersPerSecondPerSecond.of(dt.get()));
                                                            },
                                                            () -> {
                                                              smartMotorController.setMotionProfileMaxAcceleration(
                                                                  RotationsPerSecondPerSecond.of(dt.get()));
                                                            });
            break;
          }
          case MotionProfileMaxVelocity ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(distance -> {
                                                              smartMotorController.setMotionProfileMaxVelocity(MetersPerSecond.of(dt.get()));
                                                            },
                                                            () -> {
                                                              smartMotorController.setMotionProfileMaxVelocity(
                                                                  RotationsPerSecond.of(dt.get()));
                                                            });

            break;
          }
        }
      }
    }
  }

  /**
   * Close and unpublish telemetry.
   */
  public void close()
  {
    if (doubleFields != null)
    {
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().close();
      }
    }
    if (boolFields != null)
    {
      for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
      {
        entry.getValue().close();
      }
    }
  }

  /**
   * Boolean telemetry for {@link SmartMotorController}s
   */
  public enum BooleanTelemetryField
  {
    /**
     * Mechanism upper limit
     */
    MechanismUpperLimit("limits/Mechanism Upper Limit", false, false),
    /**
     * Mechanism lower limit.
     */
    MechanismLowerLimit("limits/Mechanism Lower Limit", false, false),
    /**
     * Temperature limit if available.
     */
    TemperatureLimit("limits/Temperature Limit", false, false),
    /**
     * Velocity control currently getting used.
     */
    VelocityControl("control/Velocity Control", false, false),
    /**
     * Elevator feedforward currently getting used.
     */
    ElevatorFeedForward("control/Elevator Feedforward", false, false),
    /**
     * Arm feedforward currently getting used.
     */
    ArmFeedForward("control/Arm Feedforward", false, false),
    /**
     * Simple motor feedforward currently getting used.
     */
    SimpleMotorFeedForward("control/Simple Motor Feedforward", false, false),
    /**
     * Motion profile currently getting used.
     */
    MotionProfile("control/Motion Profile", false, false),
    /**
     * Motor inversion.
     */
    MotorInversion("motor/inverted", false, true),
    /**
     * Encoder inversion.
     */
    EncoderInversion("encoder/inverted", false, true);

    /**
     * Default value of the boolean telemetry field.
     */
    private final boolean currentValue;
    /**
     * Key that the telemetry is stored at.
     */
    private final String  key;
    /**
     * Tunable field?
     */
    private final boolean tunable;

    /**
     * Create a boolean telemetry field.
     *
     * @param fieldName    Field for {@link NetworkTable}
     * @param defaultValue Default value in NT.
     * @param tunable      Tunable field.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable)
    {
      key = fieldName;
      currentValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
     *
     * @return {@link BooleanTelemetry}
     */
    public BooleanTelemetry create()
    {
      return new BooleanTelemetry(key, currentValue, this, tunable);
    }


  }

  /**
   * Double telemetry field.
   */
  public enum DoubleTelemetryField
  {
    /**
     * Motion profile maximum velocity, could be in MPS or RPS
     */
    MotionProfileMaxVelocity("closedloop/motionprofile/maxVelocity", 0, true, "velocity"),
    /**
     * Motion profile maximum accelerartion, could be in MPS^2 or RPS^2
     */
    MotionProfileMaxAcceleration("closedloop/motionprofile/maxAcceleration", 0, true, "acceleration"),
    /**
     * kS
     */
    kS("closedloop/feedforward/kS", 0, true, "none"),
    /**
     * kV
     */
    kV("closedloop/feedforward/kV", 0, true, "none"),
    /**
     * kA
     */
    kA("closedloop/feedforward/kA", 0, true, "none"),
    /**
     * kG
     */
    kG("closedloop/feedforward/kG", 0, true, "none"),
    /**
     * kP
     */
    kP("closedloop/feedback/kP", 0, true, "none"),
    /**
     * kI
     */
    kI("closedloop/feedback/kI", 0, true, "none"),
    /**
     * kD
     */
    kD("closedloop/feedback/kD", 0, true, "none"),
    /**
     * Setpoint position
     */
    TunableSetpointPosition("closedloop/setpoint/position", 0, true, "position"),
    /**
     * Setpoint position
     */
    SetpointPosition("closedloop/setpoint/position", 0, false, "position"),
    /**
     * Setpoint velocity
     */
    TunableSetpointVelocity("closedloop/setpoint/velocity", 0, true, "velocity"),
    /**
     * Setpoint velocity
     */
    SetpointVelocity("closedloop/setpoint/velocity", 0, false, "velocity"),
    /**
     * Voltage supplied to the motor.
     */
    OutputVoltage("motor/outputVoltage", 0, false, "volts"),
    /**
     * Stator current.
     */
    StatorCurrent("current/stator", 0, false, "amps"),
    /**
     * Supply current limit.
     */
    StatorCurrentLimit("current/limit/stator", 0, true, "amps"),
    /**
     * Supply current.
     */
    SupplyCurrent("current/supply", 0, false, "amps"),
    /**
     * Supply current limit.
     */
    SupplyCurrentLimit("current/limit/supply", 0, true, "amps"),
    /**
     * Motor temperature
     */
    MotorTemperature("motor/temperature", 0, false, "fahrenheit"),
    /**
     * Measurement position
     */
    MeasurementPosition("measurement/position", 0, false, "meters"),
    /**
     * Measurement velocity.
     */
    MeasurementVelocity("measurement/velocity", 0, false, "meters_per_second"),
    /**
     * Measurement lower limit.
     */
    MeasurementLowerLimit("measurement/limit/lower", 0, true, "meters"),
    /**
     * Measurement upper limit.
     */
    MeasurementUpperLimit("measurement/limit/upper", 0, true, "meters"),
    /**
     * Mechanism position in rotations.
     */
    MechanismPosition("mechanism/position", 0, false, "rotations"),
    /**
     * Mechanism velocity in rotations per second.
     */
    MechanismVelocity("mechanism/velocity", 0, false, "rotations_per_second"),
    /**
     * Mechanism lower limit in rotations.
     */
    MechanismLowerLimit("mechanism/limit/lower", 0, true, "rotations"),
    /**
     * Mechanism upper limit in rotations.
     */
    MechanismUpperLimit("mechanism/limit/upper", 0, true, "rotations"),
    /**
     * Rotor position in rotations.
     */
    RotorPosition("rotor/position", 0, false, "rotations"),
    /**
     * Rotor velocity in rotations.
     */
    RotorVelocity("rotor/velocity", 0, false, "rotations_per_second"),
    /**
     * Closed loop dutycyle ramp rate.
     */
    ClosedloopRampRate("ramprate/dutycycle/closedloop", 0, true, "none"),
    /**
     * Open loop dutycycle ramp rate.
     */
    OpenloopRampRate("ramprate/dutycycle/openloop", 0, true, "none");

    private final double  defaultVal;
    private final String  key;
    private final boolean tunable;
    private final String  unit;

    /**
     * Create double telemetry field.
     *
     * @param fieldName    NT Field Name
     * @param defaultValue Default value
     * @param tunable      Tunable places it only in the Tuning Table.
     * @param unit         Unit of the telemetry field. Special types are "position", velocity", and "acceleration".
     */
    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable, String unit)
    {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
      this.unit = unit;
    }

    /**
     * Create double telemetry field.
     *
     * @return Double telemetry field.
     */
    public DoubleTelemetry create()
    {
      return new DoubleTelemetry(key, defaultVal, this, tunable, unit);
    }

  }
}
