package yams.telemetry;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;

import java.util.Arrays;
import java.util.Map;
import java.util.Optional;
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
   * DataLog entry name
   */
  private Optional<String> dataLogName  = Optional.empty();
  /**
   * Enable telemetry over network tables.
   */
  private boolean          NT4Telemetry = true;
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
   * Set up a DataLog entry for this {@link SmartMotorController}
   *
   * @param dataLogName DataLog entry name
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withDataLogName(String dataLogName)
  {
    this.dataLogName = Optional.ofNullable(dataLogName);
    return this;
  }

  /**
   * Enable or disable NT4 Telemetry. This will not create NT4 entries and is generally only advisable during
   * competition matches.
   *
   * @param NT4Telemetry NT4 Boolean
   * @return {@link SmartMotorControllerTelemetryConfig} for chaining.
   */
  public SmartMotorControllerTelemetryConfig withNetworkTables(boolean NT4Telemetry)
  {
    this.NT4Telemetry = NT4Telemetry;
    return this;
  }

  /**
   * Disable NetworkTable output.
   *
   * @return Disable NT4 telemetry.
   */
  public SmartMotorControllerTelemetryConfig withoutNetworkTables()
  {
    this.NT4Telemetry = false;
    return this;
  }

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
        doubleFields.get(DoubleTelemetryField.TunableClosedLoopControllerSlot).enable();
        doubleFields.get(DoubleTelemetryField.MotorTemperature).enable();
        doubleFields.get(DoubleTelemetryField.MechanismLowerLimit).enable();
        doubleFields.get(DoubleTelemetryField.MechanismUpperLimit).enable();
        doubleFields.get(DoubleTelemetryField.StatorCurrentLimit).enable();
        doubleFields.get(DoubleTelemetryField.SupplyCurrentLimit).enable();
        doubleFields.get(DoubleTelemetryField.OpenloopRampRate).enable();
        doubleFields.get(DoubleTelemetryField.ClosedloopRampRate).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementLowerLimit).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementUpperLimit).enable();
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
        doubleFields.get(DoubleTelemetryField.ActiveClosedLoopControllerSlot).enable();
        doubleFields.get(DoubleTelemetryField.SetpointPosition).enable();
        doubleFields.get(DoubleTelemetryField.SetpointVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementPosition).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MeasurementAcceleration).enable();
        doubleFields.get(DoubleTelemetryField.MechanismPosition).enable();
        doubleFields.get(DoubleTelemetryField.MechanismVelocity).enable();
        doubleFields.get(DoubleTelemetryField.MechanismAcceleration).enable();
        doubleFields.get(DoubleTelemetryField.RotorPosition).enable();
        doubleFields.get(DoubleTelemetryField.RotorVelocity).enable();
        doubleFields.get(DoubleTelemetryField.ExternalEncoderPosition).enable();
        doubleFields.get(DoubleTelemetryField.ExternalEncoderVelocity).enable();
    }
    if (verbosity == TelemetryVerbosity.HIGH)
    {
      for (DoubleTelemetry dt : doubleFields.values())
      {
        if (!dt.enabled)
        {
          //System.err.println("DT " + dt.getField().name() + " is DISABLED!!");
        }
      }
      for (BooleanTelemetry dt : boolFields.values())
      {
        if (!dt.enabled)
        {
          //System.err.println("BT " + dt.getField().name() + " is DISABLED!!");
        }
      }
    }
    return this;
  }

  /**
   * Get the entry name for the smart motor controller in DataLog.
   *
   * @return DataLog entry name.
   */
  public Optional<String> getDataLogName()
  {
    return dataLogName;
  }

  /**
   * Log telemetry to NT4?
   *
   * @return should Telemetry be sent to NT4.
   */
  public boolean getNT4Enabled()
  {
    return NT4Telemetry;
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
    if (config.getSimpleFeedforward(smc.getClosedLoopControllerSlot()).isEmpty())
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
    config.getPID(smc.getClosedLoopControllerSlot()).ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kP).setDefaultValue(e.getP());
      doubleFields.get(DoubleTelemetryField.kI).setDefaultValue(e.getI());
      doubleFields.get(DoubleTelemetryField.kD).setDefaultValue(e.getD());
    });
    config.getTrapezoidProfile().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.ExponentialProfileMaxInput).disable();
      doubleFields.get(DoubleTelemetryField.ExponentialProfileKA).disable();
      doubleFields.get(DoubleTelemetryField.ExponentialProfileKV).disable();

      doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxAcceleration).enable();
      if (config.getVelocityTrapezoidalProfileInUse())
      {
        var maxJerk = RotationsPerSecondPerSecond.per(Second).of(e.maxAcceleration);
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxJerk).setDefaultValue(maxJerk.in(RPM.per(Second)
                                                                                                       .per(Second)));
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxJerk).enable();
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity).disable();
      } else if (config.getLinearClosedLoopControllerUse())
      {
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxAcceleration)
                    .setDefaultValue(e.maxAcceleration);
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity)
                    .setDefaultValue(e.maxVelocity);
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity).enable();
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxJerk).disable();
      } else
      {
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxAcceleration)
                    .setDefaultValue(RotationsPerSecondPerSecond.of(e.maxAcceleration).in(RPM.per(Minute)));
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity)
                    .setDefaultValue(RotationsPerSecond.of(e.maxVelocity).in(RPM));
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity).enable();
        doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxJerk).disable();
      }
    });
    config.getExponentialProfile().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxAcceleration).disable();
      doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxVelocity).disable();
      doubleFields.get(DoubleTelemetryField.TrapezoidalProfileMaxJerk).disable();

      doubleFields.get(DoubleTelemetryField.ExponentialProfileKA).enable();
      doubleFields.get(DoubleTelemetryField.ExponentialProfileKV).enable();
      doubleFields.get(DoubleTelemetryField.ExponentialProfileMaxInput).enable();
      var defaultkV = config.getLinearClosedLoopControllerUse() ?
                      config.convertToMechanism(Meters.of(-e.A / e.B)).in(Rotations) : (-e.A / e.B);
      var defaultkA = config.getLinearClosedLoopControllerUse() ?
                      config.convertToMechanism(Meters.of(1.0 / e.B)).in(Rotations) : (1.0 / e.B);
      var defaultMaxInput = e.maxInput;
      doubleFields.get(DoubleTelemetryField.ExponentialProfileKA).setDefaultValue(defaultkA);
      doubleFields.get(DoubleTelemetryField.ExponentialProfileKV).setDefaultValue(defaultkV);
      doubleFields.get(DoubleTelemetryField.ExponentialProfileMaxInput).setDefaultValue(defaultMaxInput);
    });
    config.getLQRClosedLoopController().ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kP).disable();
      doubleFields.get(DoubleTelemetryField.kI).disable();
      doubleFields.get(DoubleTelemetryField.kD).disable();
    });
    config.getArmFeedforward(smc.getClosedLoopControllerSlot()).ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kG).enable();
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
      doubleFields.get(DoubleTelemetryField.kG).setDefaultValue(e.getKg());
    });
    config.getElevatorFeedforward(smc.getClosedLoopControllerSlot()).ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kG).enable();
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
      doubleFields.get(DoubleTelemetryField.kG).setDefaultValue(e.getKg());
    });
    config.getSimpleFeedforward(smc.getClosedLoopControllerSlot()).ifPresent(e -> {
      doubleFields.get(DoubleTelemetryField.kG).disable();
      doubleFields.get(DoubleTelemetryField.kS).setDefaultValue(e.getKs());
      doubleFields.get(DoubleTelemetryField.kV).setDefaultValue(e.getKv());
      doubleFields.get(DoubleTelemetryField.kA).setDefaultValue(e.getKa());
    });
    if (smc.getExternalEncoderPosition().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.ExternalEncoderPosition).disable();
    }
    if (smc.getExternalEncoderVelocity().isEmpty())
    {
      doubleFields.get(DoubleTelemetryField.ExternalEncoderVelocity).disable();
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
    if (config.getArmFeedforward(smc.getClosedLoopControllerSlot()).isEmpty())
    {
      boolFields.get(BooleanTelemetryField.ArmFeedForward).disable();
    }
    if (config.getElevatorFeedforward(smc.getClosedLoopControllerSlot()).isEmpty())
    {
      boolFields.get(BooleanTelemetryField.ElevatorFeedForward).disable();
    }
    if (config.getSimpleFeedforward(smc.getClosedLoopControllerSlot()).isEmpty())
    {
      boolFields.get(BooleanTelemetryField.SimpleMotorFeedForward).disable();
    }
    if (config.getMotorInverted().isPresent())
    {
      boolFields.get(BooleanTelemetryField.MotorInversion).setDefaultValue(config.getMotorInverted().get());
    } else
    {
      boolFields.get(BooleanTelemetryField.MotorInversion).disable();
    }
    if (config.getEncoderInverted().isPresent())
    {
      boolFields.get(BooleanTelemetryField.EncoderInversion).setDefaultValue(config.getEncoderInverted().get());
    } else
    {
      boolFields.get(BooleanTelemetryField.EncoderInversion).disable();
    }
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
