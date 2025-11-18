package yams.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import java.util.Arrays;
import java.util.Map;
import java.util.stream.Collectors;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.telemetry.SmartMotorControllerTelemetry.BooleanTelemetryField;
import yams.telemetry.SmartMotorControllerTelemetry.DoubleTelemetryField;

/**
 * Smart motor controller telemetry configuration.
 */
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
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
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
        boolFields.get(BooleanTelemetryField.MotorInversion).enable();
        boolFields.get(BooleanTelemetryField.EncoderInversion).enable();
        doubleFields.get(DoubleTelemetryField.TunableSetpointPosition).enable();
        doubleFields.get(DoubleTelemetryField.TunableSetpointVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MotorTemperature).enable();
        doubleFields.get(DoubleTelemetryField.MechanismLowerLimit).enable();
        doubleFields.get(DoubleTelemetryField.MechanismUpperLimit).enable();
        doubleFields.get(DoubleTelemetryField.StatorCurrentLimit).enable();
        doubleFields.get(DoubleTelemetryField.SupplyCurrentLimit).enable();
        doubleFields.get(DoubleTelemetryField.OpenloopRampRate).enable();
        doubleFields.get(DoubleTelemetryField.ClosedloopRampRate).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementLowerLimit).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementUpperLimit).enable();
        doubleFields.get(DoubleTelemetryField.MotionProfileMaxAcceleration).enable();
        doubleFields.get(DoubleTelemetryField.MotionProfileMaxVelocity).enable();
        doubleFields.get(DoubleTelemetryField.kS).enable();
        doubleFields.get(DoubleTelemetryField.kV).enable();
        doubleFields.get(DoubleTelemetryField.kG).enable();
        doubleFields.get(DoubleTelemetryField.kA).enable();
        doubleFields.get(DoubleTelemetryField.kP).enable();
        doubleFields.get(DoubleTelemetryField.kI).enable();
        doubleFields.get(DoubleTelemetryField.kD).enable();
      case MID:
        doubleFields.get(DoubleTelemetryField.OutputVoltage).enable();
        doubleFields.get(DoubleTelemetryField.StatorCurrent).enable();
        doubleFields.get(DoubleTelemetryField.SupplyCurrent).enable();
      case LOW:
        doubleFields.get(DoubleTelemetryField.SetpointPosition).enable();
        doubleFields.get(DoubleTelemetryField.SetpointVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementPosition).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MechanismPosition).enable();
        doubleFields.get(DoubleTelemetryField.MechanismVelocity).enable();
        doubleFields.get(DoubleTelemetryField.RotorPosition).enable();
        doubleFields.get(DoubleTelemetryField.RotorVelocity).enable();
    }
    if (verbosity == TelemetryVerbosity.HIGH)
    {
      for (DoubleTelemetry dt : doubleFields.values())
      {
        if (!dt.enabled)
        {
          System.err.println("DT " + dt.getField().name() + " is DISABLED!!");
        }
      }
      for (BooleanTelemetry dt : boolFields.values())
      {
        if (!dt.enabled)
        {
          System.err.println("BT " + dt.getField().name() + " is DISABLED!!");
        }
      }
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
    var config         = smc.getConfig();
    var unsupTelemetry = smc.getUnsupportedTelemetryFields();
    unsupTelemetry.getFirst().ifPresent(btList -> {
      for (BooleanTelemetryField bt : btList)
      {
        boolFields.get(bt).disable();
      }
    });
    unsupTelemetry.getSecond().ifPresent(dtList -> {
      for (DoubleTelemetryField dt : dtList)
      {
        doubleFields.get(dt).disable();
      }
    });
    if (smc.getSupplyCurrent().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.SupplyCurrent).disable();
      doubleFields.get(DoubleTelemetryField.SupplyCurrentLimit).disable();
    }
    if (config.getSimpleFeedforward().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.kG).disable();
    }
    if (config.getMechanismCircumference().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.MeasurementLowerLimit).disable();
      doubleFields.get(DoubleTelemetryField.MeasurementUpperLimit).disable();
      doubleFields.get(DoubleTelemetryField.MeasurementPosition).disable();
      doubleFields.get(DoubleTelemetryField.MeasurementVelocity).disable();
    } else
    {
      config.getMechanismUpperLimit()
            .ifPresent(upperLimit -> doubleFields.get(DoubleTelemetryField.MeasurementUpperLimit)
                                                 .setDefaultValue(config.convertFromMechanism(upperLimit).in(Meters)));
      config.getMechanismLowerLimit().ifPresent(limit -> doubleFields.get(DoubleTelemetryField.MeasurementLowerLimit)
                                                                     .setDefaultValue(config.convertFromMechanism(limit)
                                                                                            .in(Meters)));
    }
    config.getMechanismUpperLimit().ifPresent(limit -> doubleFields.get(DoubleTelemetryField.MechanismUpperLimit)
                                                                   .setDefaultValue(limit.in(Degrees)));
    config.getMechanismLowerLimit().ifPresent(limit -> doubleFields.get(DoubleTelemetryField.MechanismLowerLimit)
                                                                   .setDefaultValue(limit.in(Degrees)));
    config.getSupplyStallCurrentLimit().ifPresent(e -> doubleFields.get(DoubleTelemetryField.SupplyCurrentLimit)
                                                                   .setDefaultValue(e));
    config.getStatorStallCurrentLimit().ifPresent(e -> doubleFields.get(DoubleTelemetryField.StatorCurrentLimit)
                                                                   .setDefaultValue(e));
    config.getSimpleClosedLoopController().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kP).setDefaultValue(e.getP());
      doubleFields.get(DoubleTelemetryField.kI).setDefaultValue(e.getI());
      doubleFields.get(DoubleTelemetryField.kD).setDefaultValue(e.getD());
    });
    config.getClosedLoopController().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kP).setDefaultValue(e.getP());
      doubleFields.get(DoubleTelemetryField.kI).setDefaultValue(e.getI());
      doubleFields.get(DoubleTelemetryField.kD).setDefaultValue(e.getD());
      doubleFields.get(DoubleTelemetryField.MotionProfileMaxAcceleration)
                  .setDefaultValue(RotationsPerSecondPerSecond.of(e.getConstraints().maxAcceleration)
                                                              .in(DegreesPerSecondPerSecond));
      doubleFields.get(DoubleTelemetryField.MotionProfileMaxVelocity)
                  .setDefaultValue(RotationsPerSecond.of(e.getConstraints().maxVelocity).in(DegreesPerSecond));
    });
    config.getExponentiallyProfiledClosedLoopController().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kP).setDefaultValue(e.getP());
      doubleFields.get(DoubleTelemetryField.kI).setDefaultValue(e.getI());
      doubleFields.get(DoubleTelemetryField.kD).setDefaultValue(e.getD());
      // TODO: Add kV and kA
    });
    config.getArmFeedforward().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kG).enable();
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
      doubleFields.get(DoubleTelemetryField.kG).setDefaultValue(e.getKg());
    });
    config.getElevatorFeedforward().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kG).enable();
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
      doubleFields.get(DoubleTelemetryField.kG).setDefaultValue(e.getKg());
    });
    config.getSimpleFeedforward().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
    });
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
    if (config.getArmFeedforward().isEmpty())
    {
      boolFields.get(BooleanTelemetryField.ArmFeedForward).disable();
    }
    if (config.getElevatorFeedforward().isEmpty())
    {
      boolFields.get(BooleanTelemetryField.ElevatorFeedForward).disable();
    }
    if (config.getSimpleFeedforward().isEmpty())
    {
      boolFields.get(BooleanTelemetryField.SimpleMotorFeedForward).disable();
    }
    boolFields.get(BooleanTelemetryField.MotorInversion).setDefaultValue(config.getMotorInverted());
    boolFields.get(BooleanTelemetryField.EncoderInversion).setDefaultValue(config.getEncoderInverted());
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
