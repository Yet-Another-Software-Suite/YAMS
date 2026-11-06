// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import com.revrobotics.spark.A301;
import java.util.ArrayList;
import java.util.List;
import java.util.function.UnaryOperator;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.RoboRioSim;
import org.wpilib.util.Pair;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.local.A301Wrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests the {@link A301Wrapper}: the options the A301 does not have are rejected, its closed loop
 * runs on the SystemCore, and its absolute encoder reads in the wrapping range.
 */
public class A301WrapperTest {
  private static final ClosedLoopCases.Controller kA301 =
      new ClosedLoopCases.Controller(
          "A301",
          YMotors.getA301(1),
          ClosedLoopCases.kA301Ratio,
          4,
          false,
          (motor, cfg) -> new A301Wrapper(DeviceCreator.createA301(), cfg));

  /** A301s created by a test, closed after it. */
  private final List<A301> m_devices = new ArrayList<>();

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
    RoboRioSim.setVInVoltage(12.0);
  }

  @AfterEach
  void endTest() {
    m_devices.forEach(A301::close);
    m_devices.clear();
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  private static yams.commands2.config.SmartMotorControllerConfig config(String name) {
    return new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        // The A301's 64:1 gearbox.
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(64)))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(10, 0, 0)
        .withTelemetry("A301WrapperTest " + name, TelemetryVerbosity.LOW);
  }

  private A301 createA301() {
    final A301 a301 = DeviceCreator.createA301();
    m_devices.add(a301);
    return a301;
  }

  private void assertRejected(
      Class<? extends RuntimeException> exception,
      String name,
      UnaryOperator<yams.commands2.config.SmartMotorControllerConfig> option) {
    assertThrows(exception, () -> new A301Wrapper(createA301(), option.apply(config(name))), name);
  }

  @Test
  void rejectsWhatTheA301DoesNotHave() {
    final var configException = SmartMotorControllerConfigurationException.class;
    assertRejected(configException, "stator limit", cfg -> cfg.withStatorCurrentLimit(Amps.of(40)));
    assertRejected(configException, "supply limit", cfg -> cfg.withSupplyCurrentLimit(Amps.of(40)));
    assertRejected(configException, "open ramp", cfg -> cfg.withOpenLoopRampRate(Seconds.of(0.1)));
    assertRejected(
        configException, "closed ramp", cfg -> cfg.withClosedLoopRampRate(Seconds.of(0.1)));
    assertRejected(
        configException, "voltage comp", cfg -> cfg.withVoltageCompensation(Volts.of(12)));
    assertRejected(configException, "vendor config", cfg -> cfg.withVendorConfig(new Object()));
    assertRejected(
        configException, "followers", cfg -> cfg.withFollowers(Pair.of(createA301(), false)));
    assertRejected(
        configException,
        "zero offset",
        cfg -> cfg.withExternalEncoderZeroOffset(Rotations.of(0.1)));
    // Only the A301's own absolute encoder can be its external encoder.
    assertRejected(
        IllegalArgumentException.class,
        "another A301",
        cfg -> cfg.withExternalEncoder(createA301()).withUseExternalFeedbackEncoder(true));
  }

  @Test
  void closedLoopRunsOnTheSystemCore() {
    final SmartMotorController smc = ClosedLoopCases.create(kA301, config("closed loop"));
    m_devices.add((A301) smc.getMotorController());
    ClosedLoopCases.run(kA301, smc, 3, () -> smc.setPosition(Rotations.of(0.25)), time -> {});
    assertEquals(0.25, smc.getMechanismPosition().in(Rotations), 0.02);
    ClosedLoopCases.close(smc);
  }

  @Test
  void absoluteEncoderReadsInTheWrappingRange() {
    final A301 zeroToOne = createA301();
    final SmartMotorController defaultRange =
        new A301Wrapper(
            zeroToOne,
            config("default range")
                .withExternalEncoder(zeroToOne)
                .withUseExternalFeedbackEncoder(true)
                .withStartingPosition(Rotations.of(-0.2)));
    assertEquals(0.8, defaultRange.getMechanismPosition().in(Rotations), 1e-6);

    final A301 centered = createA301();
    final SmartMotorController wrapped =
        new A301Wrapper(
            centered,
            config("centered range")
                .withExternalEncoder(centered)
                .withUseExternalFeedbackEncoder(true)
                .withWrapping(Rotations.of(-0.5), Rotations.of(0.5))
                .withStartingPosition(Rotations.of(-0.2)));
    assertEquals(-0.2, wrapped.getMechanismPosition().in(Rotations), 1e-6);
    assertTrue(wrapped.getExternalEncoderMechanismPosition().isPresent());
    defaultRange.close();
    wrapped.close();
  }

  @Test
  void motorIsTheA301s() {
    // The A301's motor: 13,500 RPM free, and 5.27 Nm at peak power through 135:1.
    final DCMotor motor = YMotors.getA301(1);
    assertEquals(13500, motor.freeSpeed * 60 / (2 * Math.PI), 1e-6);
    assertEquals(5.27, motor.stallTorque * 135 / 2, 1e-9);

    final A301Wrapper smc = new A301Wrapper(createA301(), config("motor"));
    assertEquals(motor.freeSpeed, smc.getDCMotor().freeSpeed, 1e-9);
    smc.close();
  }
}
