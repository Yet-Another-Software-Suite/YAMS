// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.local.SparkWrapper;

/** Tests {@code SmartMechanism.getMeasurementSetpoint()} on an {@link Elevator}. */
public class MeasurementSetpointTest {
  @Test
  void measurementSetpointIsTheMechanismSetpointAsADistance() {
    MockHardwareExtension.beforeAll();
    // An ID above the ones DeviceCreator hands out, so the other tests keep the IDs they had.
    final SparkMax spark = new SparkMax(99, MotorType.kBrushless);
    final SmartMotorController smc =
        new SparkWrapper(
            spark,
            DCMotor.getNEO(1),
            new SmartMotorControllerConfig(new SmartMotorControllerTestSubsystem())
                .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
                .withClosedLoopController(4, 0, 0)
                .withSoftLimits(Meters.of(0), Meters.of(2))
                .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                .withStartingPosition(Meters.of(0))
                .withStatorCurrentLimit(Amps.of(40))
                .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
                .withControlMode(ControlMode.CLOSED_LOOP));
    try {
      final Elevator elevator =
          new Elevator(
              new ElevatorConfig()
                  .withHardLimits(Meters.of(0), Meters.of(3))
                  .withCarriageWeight(Pounds.of(16)),
              smc);
      assertTrue(
          elevator.getMeasurementSetpoint().isEmpty(), "there is no setpoint before one is set");
      smc.setPosition(Meters.of(0.75));
      assertEquals(0.75, elevator.getMeasurementSetpoint().orElseThrow().in(Meters), 1e-6);
      assertEquals(
          smc.getConfig().convertFromMechanism(elevator.getMechanismSetpoint().orElseThrow()).in(Meters),
          elevator.getMeasurementSetpoint().orElseThrow().in(Meters),
          1e-9);
    } finally {
      smc.close();
      spark.close();
      MockHardwareExtension.afterAll();
    }
  }
}
