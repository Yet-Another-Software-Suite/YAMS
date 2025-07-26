package yams.telemetry;

import edu.wpi.first.networktables.NetworkTable;
import java.util.Map;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;

import static edu.wpi.first.units.Units.*;

public class SmartMotorControllerTelemetry
{


  /**
   * {@link TelemetryVerbosity} for the {@link SmartMotorController}
   */
  private final TelemetryVerbosity                           verbosity = TelemetryVerbosity.HIGH;
  private       Map<DoubleTelemetryField, DoubleTelemetry>   doubleFields;
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
   * @param smartMotorController {@link SmartMotorController} used to determine what telemetry features can and should be disabled bc they are not available.
   * @param publishTable {@link NetworkTable} that holds all of the output fields.
   * @param tuningTable  {@link NetworkTable} that holds all of the tuning fields.
   * @param config       {@link SmartMotorControllerTelemetryConfig} to apply.
   */
  public void setupTelemetry(SmartMotorController smartMotorController, NetworkTable publishTable, NetworkTable tuningTable,
                             SmartMotorControllerTelemetryConfig config)
  {
    if (!publishTable.equals(this.dataNetworkTable))
    {
      dataNetworkTable = publishTable;
      this.config = config;
      doubleFields = config.getDoubleFields(smartMotorController);
      boolFields = config.getBoolFields(smartMotorController);
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }
      for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
      {
        entry.getValue().setupNetworkTables(dataNetworkTable, tuningNetworkTable);
      }

    }
  }


  /**
   * Publish {@link SmartMotorController} telemetry to {@link NetworkTable}
   */
  public void publish(SmartMotorController smc)
  {
    SmartMotorControllerConfig cfg = smc.getConfig();
    for (Map.Entry<BooleanTelemetryField, BooleanTelemetry> entry : boolFields.entrySet())
    {
      BooleanTelemetry bt = entry.getValue();
      if(!bt.enabled)
        continue;
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
      if(!dt.enabled)
        continue;
      switch (dt.getField())
      {
        case SetpointPosition ->
        {
          smc.getMechanismPositionSetpoint().ifPresent(mechSetpoint -> dt.set(
              cfg.getMechanismCircumference().isPresent() ? cfg.convertFromMechanism(mechSetpoint).in(Meters)
                                                          : mechSetpoint.in(Rotations)));
        }
        case SetpointVelocity ->
        {
          smc.getMechanismSetpointVelocity().ifPresent(mechSetpoint ->
                                                           dt.set(cfg.getMechanismCircumference().isPresent() ?
                                                                  cfg.convertFromMechanism(mechSetpoint)
                                                                     .in(MetersPerSecond) :
                                                                  mechSetpoint.in(RotationsPerSecond)));
        }
        case FeedforwardVoltage ->
        {
          // TODO: Remove
//          smc.getFeedforwardVoltage();
        }
        case PIDOutputVoltage ->
        {
          // TODO: Remove
//          smc.getPIDOutputVoltage();
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
   * @param smartMotorController {@link SmartMotorController} to control.
   */
  public void applyTuningValues(SmartMotorController smartMotorController)
  {

    SmartMotorControllerConfig cfg = smartMotorController.getConfig();
    for (Map.Entry<DoubleTelemetryField, DoubleTelemetry> entry : doubleFields.entrySet())
    {
      DoubleTelemetry dt = entry.getValue();
      if (dt.tunable())
      {
        switch (dt.getField())
        {
          case SetpointPosition ->
          {
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setPosition(Meters.of(
                dt.get())), () -> smartMotorController.setPosition(Radians.of(dt.get())));
          }
          case SetpointVelocity ->
          {
            if(dt.get() == 0 && smartMotorController.getMechanismSetpointVelocity().isEmpty())
              return;
            cfg.getMechanismCircumference().ifPresentOrElse(circumference -> smartMotorController.setVelocity(
                MetersPerSecond.of(
                    dt.get())), () -> smartMotorController.setVelocity(dt.get() == 0 ? null : RadiansPerSecond.of(dt.get())));
          }
        }
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
    MotionProfile("control/Motion Profile", false, false);

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
     * @param fieldName Field for {@link NetworkTable}
     * @param defaultValue Default value in NT.
     * @param tunable Tunable field.
     */
    BooleanTelemetryField(String fieldName, boolean defaultValue, boolean tunable)
    {
      key = fieldName;
      currentValue = defaultValue;
      this.tunable = tunable;
    }

    /**
     * Create a {@link BooleanTelemetry} object for non-static usage.
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
     * Setpoint position
     */
    SetpointPosition("closedloop/setpoint/position", 0, true),
    /**
     * Setpoint velocity
     */
    SetpointVelocity("closedloop/setpoint/velocity", 0, true),
    /**
     * Feedforward voltage.
     */
    FeedforwardVoltage("closedloop/voltage/feedforward", 0, false),
    /**
     * PID output voltage
     */
    PIDOutputVoltage("closedloop/voltage/feedback", 0, false),
    /**
     * Voltage supplied to the motor.
     */
    OutputVoltage("Motor Voltage", 0, false),
    /**
     * Stator current.
     */
    StatorCurrent("current/stator", 0, false),
    /**
     * Supply current.
     */
    SupplyCurrent("current/supply", 0, false),
    /**
     * Motor temperature
     */
    MotorTemperature("Motor Temperature", 0, false),
    /**
     * Measurement position
     */
    MeasurementPosition("measurement/position (meters)", 0, false),

    MeasurementVelocity("measurement/velocity (meters per second)", 0, false),
    MechanismPosition("mechanism/position (rotations)", 0, false),
    MechanismVelocity("mechanism/velocity (rotations per second)", 0, false),
    RotorPosition("rotor/position (rotations)", 0, false),
    RotorVelocity("rotor/velocity (rotations per second)", 0, false);

    private final double  defaultVal;
    private final String  key;
    private final boolean tunable;

    DoubleTelemetryField(String fieldName, double defaultValue, boolean tunable)
    {
      key = fieldName;
      defaultVal = defaultValue;
      this.tunable = tunable;
    }

    public DoubleTelemetry create()
    {
      return new DoubleTelemetry(key, defaultVal, this, tunable);
    }

  }
}
