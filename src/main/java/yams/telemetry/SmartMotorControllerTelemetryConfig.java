package yams.telemetry;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;

import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

public class SmartMotorControllerTelemetryConfig
{

  /**
   * {@link BooleanTelemetryField}s to enable or disable.
   */
  private final Map<BooleanTelemetryField, BooleanTelemetry> boolFields   = Arrays.stream(BooleanTelemetryField.values())
                                                                                  .collect(
                                                                                      Collectors.toMap(e -> e,
                                                                                                       BooleanTelemetryField::create));
  /**
   * {@link DoubleTelemetryField} to enable or disable.
   */
  private final Map<DoubleTelemetryField, DoubleTelemetry>   doubleFields = Arrays.stream(DoubleTelemetryField.values())
                                                                                  .collect(Collectors.toMap(e -> e,
                                                                                                            DoubleTelemetryField::create));

  /**
   * Setup with {@link TelemetryVerbosity}
   *
   * @param verbosity {@link TelemetryVerbosity} to use.
   */
  public SmartMotorControllerTelemetryConfig withTelemetryVerbosity(TelemetryVerbosity verbosity)
  {
    switch (verbosity)
    {
      case HIGH:
        boolFields.get(BooleanTelemetryField.MechanismLowerLimit).enable();
        boolFields.get(BooleanTelemetryField.MechanismUpperLimit).enable();

        boolFields.get(BooleanTelemetryField.TemperatureLimit).enable();
        boolFields.get(BooleanTelemetryField.VelocityControl).enable();
        boolFields.get(BooleanTelemetryField.ElevatorFeedForward).enable();
        boolFields.get(BooleanTelemetryField.ArmFeedForward).enable();
        boolFields.get(BooleanTelemetryField.SimpleMotorFeedForward).enable();
        boolFields.get(BooleanTelemetryField.MotionProfile).enable();
        doubleFields.get(DoubleTelemetryField.SetpointPosition).enable();
        doubleFields.get(DoubleTelemetryField.SetpointVelocity).enable();
        doubleFields.get(DoubleTelemetryField.FeedforwardVoltage).enable();
        doubleFields.get(DoubleTelemetryField.PIDOutputVoltage).enable();
        doubleFields.get(DoubleTelemetryField.StatorCurrent).enable();
        doubleFields.get(DoubleTelemetryField.SupplyCurrent).enable();
        doubleFields.get(DoubleTelemetryField.MotorTemperature).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementPosition).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MechanismPosition).enable();
        doubleFields.get(DoubleTelemetryField.MechanismVelocity).enable();
        doubleFields.get(DoubleTelemetryField.RotorPosition).enable();
        doubleFields.get(DoubleTelemetryField.RotorVelocity).enable();
      case MID:
      case LOW:
    }
    return this;
  }

  /**
   * Get the configured double fields.
   *
   * @param smc {@link SmartMotorController} used to disable unavailable telemetry for certain motor controllers.
   * @return Configured {@link DoubleTelemetry} for each {@link DoubleTelemetryField}
   */
  public Map<DoubleTelemetryField, DoubleTelemetry> getDoubleFields(SmartMotorController smc)
  {
    if(smc.getSupplyCurrent().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.SupplyCurrent).disable();
    }
    return doubleFields;
  }

  /**
   * Get the configured bool fields.
   *
   * @param smc {@link SmartMotorController} used to disable unavailable telemetry for certain motor controllers.
   * @return Configured {@link BooleanTelemetry} for each {@link BooleanTelemetryField}.
   */
  public Map<BooleanTelemetryField, BooleanTelemetry> getBoolFields(SmartMotorController smc)
  {
    var config = smc.getConfig();
    if(config.getArmFeedforward().isEmpty())
      boolFields.get(BooleanTelemetryField.ArmFeedForward).disable();
    if(config.getElevatorFeedforward().isEmpty())
      boolFields.get(BooleanTelemetryField.ElevatorFeedForward).disable();
    if(config.getSimpleFeedforward().isEmpty())
      boolFields.get(BooleanTelemetryField.SimpleMotorFeedForward).disable();
    return boolFields;
  }

  /**
   * Enables the mechanism lower limit logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMechanismLowerLimit()
  {
    boolFields.get(BooleanTelemetryField.MechanismLowerLimit).enable();
    return this;
  }

  /**
   * Enables the mechanism upper limit logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMechanismUpperLimit()
  {
    boolFields.get(BooleanTelemetryField.MechanismUpperLimit).enable();
    return this;
  }

  /**
   * Enables the temperature limit logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withTemperatureLimit()
  {
    boolFields.get(BooleanTelemetryField.TemperatureLimit).enable();
    return this;
  }

  /**
   * Enables the velocity control mode logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withVelocityControl()
  {
    boolFields.get(BooleanTelemetryField.VelocityControl).enable();
    return this;
  }

  /**
   * Enables the elevator feedforward logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withElevatorFeedforward()
  {
    boolFields.get(BooleanTelemetryField.ElevatorFeedForward).enable();
    return this;
  }

  /**
   * Enables the arm feedforward logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withArmFeedforward()
  {
    boolFields.get(BooleanTelemetryField.ArmFeedForward).enable();
    return this;
  }

  /**
   * Enables the simple feedforward logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withSimpleFeedforward()
  {
    boolFields.get(BooleanTelemetryField.SimpleMotorFeedForward).enable();
    return this;
  }

  /**
   * Enables the motion profile logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMotionProfile()
  {
    boolFields.get(BooleanTelemetryField.MotionProfile).enable();
    return this;
  }

  /**
   * Enables the setpoint position logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withSetpointPosition()
  {
    doubleFields.get(DoubleTelemetryField.SetpointPosition).enable();
    return this;
  }

  /**
   * Enables the setpoint velocity logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withSetpointVelocity()
  {
    doubleFields.get(DoubleTelemetryField.SetpointVelocity).enable();
    return this;
  }

  /**
   * Enables the feedforward voltage logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withFeedforwardVoltage()
  {
    doubleFields.get(DoubleTelemetryField.FeedforwardVoltage).enable();
    return this;
  }

  /**
   * Enables the pid output voltage logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withPidOutputVoltage()
  {
    doubleFields.get(DoubleTelemetryField.PIDOutputVoltage).enable();
    return this;
  }

  /**
   * Enables the output voltage logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withOutputVoltage()
  {
    doubleFields.get(DoubleTelemetryField.OutputVoltage).enable();
    return this;
  }

  /**
   * Enables the stator current logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withStatorCurrent()
  {
    doubleFields.get(DoubleTelemetryField.StatorCurrent).enable();
    return this;
  }

  /**
   * Enables the temperature logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withTemperature()
  {
    doubleFields.get(DoubleTelemetryField.MotorTemperature).enable();
    return this;
  }

  /**
   * Enables the distance logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMeasurementPosition()
  {
    doubleFields.get(DoubleTelemetryField.MeasurementPosition).enable();
    return this;
  }

  /**
   * Enables the linear velocity logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMeasurementVelocity()
  {
    doubleFields.get(DoubleTelemetryField.MeasurementVelocity).enable();
    return this;
  }

  /**
   * Enables the mechanism position logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMechanismPosition()
  {
    doubleFields.get(DoubleTelemetryField.MechanismPosition).enable();
    return this;
  }

  /**
   * Enables the mechanism velocity logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withMechanismVelocity()
  {
    doubleFields.get(DoubleTelemetryField.MechanismVelocity).enable();
    return this;
  }

  /**
   * Enables the rotor position logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withRotorPosition()
  {
    doubleFields.get(DoubleTelemetryField.RotorPosition).enable();
    return this;
  }

  /**
   * Enables the rotor velocity logging if available.
   *
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withRotorVelocity()
  {
    doubleFields.get(DoubleTelemetryField.RotorVelocity).enable();
    return this;
  }
}
