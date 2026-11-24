// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.A301;
import com.revrobotics.spark.SparkMax;
import com.thrifty.nova.Nova;
import java.util.Arrays;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.preferences.Preferences;
import org.wpilib.simulation.AlertSim;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.A301Wrapper;
import yams.core.motorcontrollers.local.NovaWrapper;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;

/**
 * Each motor controller warns in simulation when it builds its simulation from the default moment
 * of inertia, and not when one is set. Motor controllers that share a telemetry name, as a swerve
 * drive's modules do, each get their own alert.
 */
class DefaultMomentOfInertiaAlertTest {
  private static int offset = 0;

  private static SmartMotorControllerConfig config(String label, boolean setMomentOfInertia) {
    SmartMotorControllerConfig cfg = (SmartMotorControllerConfig) new SmartMotorControllerConfig()
        .withClosedLoopController(4, 0, 0)
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0, 0.02))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withStartingPosition(Degrees.of(0))
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withTelemetry(label, TelemetryVerbosity.HIGH);
    if (setMomentOfInertia) {
      cfg.withMomentOfInertia(Inches.of(4), Pounds.of(1));
    }
    return cfg;
  }

  static Stream<Arguments> motorControllers() {
    return Stream.of(
        Arguments.of("Spark", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg)),
        Arguments.of("TalonFX", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg)),
        Arguments.of("TalonFXS", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg)),
        Arguments.of("Nova", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new NovaWrapper(DeviceCreator.createNova(), DCMotor.getNEO(1),
                DeviceCreator.withNovaProfile(cfg, false))),
        Arguments.of("A301", (Function<SmartMotorControllerConfig, SmartMotorController>)
            cfg -> new A301Wrapper(DeviceCreator.createA301(), DeviceCreator.forA301(cfg))));
  }

  /** How many active default moment of inertia alerts there are for a telemetry name. */
  private static long activeAlerts(String label) {
    return Arrays.stream(AlertSim.getActive())
        .filter(alert -> alert.group.equals("YAMS")
            && alert.id.endsWith("_DefaultMomentOfInertia_" + label))
        .count();
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("motorControllers")
  void warnsOnlyForTheDefault(String type,
      Function<SmartMotorControllerConfig, SmartMotorController> create) {
    String unset = type + "(defaultMOI)[" + (++offset) + "]";
    String set = type + "(setMOI)[" + (++offset) + "]";
    SmartMotorController withDefault = create.apply(config(unset, false));
    SmartMotorController withMomentOfInertia = create.apply(config(set, true));
    try {
      assertEquals(1, activeAlerts(unset), type + " with the default moment of inertia");
      assertEquals(0, activeAlerts(set), type + " with a moment of inertia");
    } finally {
      close(withDefault);
      close(withMomentOfInertia);
    }
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("motorControllers")
  void motorControllersThatShareATelemetryNameEachWarn(String type,
      Function<SmartMotorControllerConfig, SmartMotorController> create) {
    // A swerve drive names every module's drive motor the same.
    String shared = type + "(driveMotor)[" + (++offset) + "]";
    SmartMotorController first = create.apply(config(shared, false));
    SmartMotorController second = create.apply(config(shared, false));
    try {
      assertEquals(2, activeAlerts(shared), type + " motor controllers sharing a telemetry name");
    } finally {
      close(first);
      close(second);
    }
  }

  private static void close(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem) ((SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    smc.close();
    DeviceCreator.silence(smc);
    Object motor = smc.getMotorController();
    if (motor instanceof SparkMax spark) {
      spark.close();
    } else if (motor instanceof TalonFXS talon) {
      talon.close();
    } else if (motor instanceof TalonFX talon) {
      talon.close();
    } else if (motor instanceof Nova nova) {
      nova.close();
    } else if (motor instanceof A301 a301) {
      a301.close();
    }
  }

  @BeforeEach
  void beforeTest() {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void afterTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
