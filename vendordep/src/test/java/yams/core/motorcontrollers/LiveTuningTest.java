// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;
import static org.wpilib.units.Units.MetersPerSecondPerSecond;
import static org.wpilib.units.Units.RPM;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.RotationsPerSecondPerSecond;
import static org.wpilib.units.Units.Second;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.MethodSource;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.trajectory.ExponentialProfile;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.preferences.Preferences;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.gearing.GearBox;
import yams.core.gearing.MechanismGearing;
import yams.core.motorcontrollers.SmartMotorController.ClosedLoopControllerSlot;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.motorcontrollers.remote.TalonFXSWrapper;
import yams.core.motorcontrollers.remote.TalonFXWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.helpers.DeviceCreator;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;

/**
 * Tests live tuning on every motor controller wrapper and every tunable closed loop controller, for
 * rotational and linear mechanisms: the tuning table shows the values given to the
 * {@link SmartMotorControllerConfig}, and values entered there are applied as given. The config
 * then holds exactly the entered values, and the motor controller holds them converted to its own
 * units.
 *
 * <p>Gains are in volts per mechanism rotation, or per meter for a linear closed loop controller;
 * trapezoidal profiles are shown in RPM and RPM per second, or meters per second and meters per
 * second squared; exponential profiles in their own units, like their constraints.
 */
public class LiveTuningTest {
  private static final double kTolerance = 1e-9;
  private static final MechanismGearing kGearing = new MechanismGearing(GearBox.fromReductionStages(3, 4));
  /** Meters per mechanism rotation of the linear mechanisms. */
  private static final double kCircumferenceMeters = 0.2;

  @BeforeEach
  void startTest() {
    MockHardwareExtension.beforeAll();
  }

  @AfterEach
  void endTest() {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
  }

  /** The tunable closed loop controllers. */
  private enum Controller {
    /** A PID controller. */
    PID,
    /** A PID controller following a trapezoidal profile. */
    TRAPEZOIDAL_PROFILE,
    /** A PID controller following an exponential profile. */
    EXPONENTIAL_PROFILE
  }

  /** A motor controller to test, created from its config when the test runs. */
  private record Case(String name, Controller controller, boolean linear,
                      Function<SmartMotorControllerConfig, SmartMotorController> create) {
    @Override
    public String toString() {
      return name;
    }
  }

  private static Stream<Case> createControllers() {
    final List<Case> cases = new ArrayList<>();
    for (Controller controller : Controller.values()) {
      for (boolean linear : new boolean[] {false, true}) {
        final String suffix = " " + controller + (linear ? " linear" : "");
        cases.add(new Case("SparkMax" + suffix, controller, linear,
            cfg -> new SparkWrapper(DeviceCreator.createSparkMax(), DCMotor.getNEO(1), cfg)));
        cases.add(new Case("SparkFlex" + suffix, controller, linear,
            cfg -> new SparkWrapper(DeviceCreator.createSparkFlex(), DCMotor.getNeoVortex(1), cfg)));
        cases.add(new Case("TalonFX" + suffix, controller, linear,
            cfg -> new TalonFXWrapper(DeviceCreator.createTalonFX(), DCMotor.getKrakenX60(1), cfg)));
        cases.add(new Case("TalonFXS" + suffix, controller, linear,
            cfg -> new TalonFXSWrapper(DeviceCreator.createTalonFXS(), DCMotor.getNEO(1), cfg)));
      }
    }
    return cases.stream();
  }

  private static SmartMotorControllerConfig config(Case testCase) {
    SmartMotorControllerConfig config = new yams.commands2.config.SmartMotorControllerConfig()
        .withSubsystem(new SmartMotorControllerTestSubsystem())
        .withGearing(kGearing)
        .withStatorCurrentLimit(Amps.of(40))
        .withIdleMode(MotorMode.BRAKE)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(1.5, 0.25, 0.125)
        .withFeedforward(new SimpleMotorFeedforward(0.2, 0.75, 0.05))
        .withTelemetry("LiveTuningTest " + testCase.name(), TelemetryVerbosity.HIGH);
    if (testCase.linear()) {
      config = config.withMechanismCircumference(Meters.of(kCircumferenceMeters)).withLinearClosedLoopController(true);
    }
    return switch (testCase.controller()) {
      case PID -> config;
      case TRAPEZOIDAL_PROFILE -> testCase.linear()
                                  ? config.withTrapezoidalProfile(MetersPerSecond.of(1.5), MetersPerSecondPerSecond.of(3))
                                  : config.withTrapezoidalProfile(RotationsPerSecond.of(2), RotationsPerSecondPerSecond.of(4));
      case EXPONENTIAL_PROFILE -> config.withExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(12, 1.5, 0.3));
    };
  }

  private static void closeSmc(SmartMotorController smc) {
    SmartMotorControllerTestSubsystem subsys =
        (SmartMotorControllerTestSubsystem)
            ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem();
    SmartMotorControllerCommandRegistry.removeCommands(subsys);
    CommandScheduler.getInstance().unregisterSubsystem(subsys);
    subsys.close();
    smc.close();
    DeviceCreator.silence(smc);

    // Simulated devices keep their configuration for the rest of the test run, and YAMS starts from a
    // device's previous configuration, so put every device back to factory defaults for the tests
    // that reuse its ID.
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax sparkMax) {
      sparkMax.configure(new SparkMaxConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      sparkMax.close();
    } else if (motorController instanceof SparkFlex sparkFlex) {
      sparkFlex.configure(new SparkFlexConfig(), ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
      sparkFlex.close();
    } else if (motorController instanceof TalonFXS talonFXS) {
      talonFXS.getConfigurator().apply(new TalonFXSConfiguration());
      talonFXS.close();
    } else if (motorController instanceof TalonFX talonFX) {
      talonFX.getConfigurator().apply(new TalonFXConfiguration());
      talonFX.close();
    }
  }

  private static void assertShown(NetworkTable tuning, String key, double expected, String name) {
    assertEquals(expected, tuning.getEntry(key).getDouble(Double.NaN), kTolerance, name + ": " + key + " shown");
  }

  private static void enter(NetworkTable tuning, String key, double value) {
    tuning.getEntry(key).setDouble(value);
  }

  /** The kP on the motor controller itself, in its own units, waiting for an asynchronous apply. */
  private static double deviceKp(SmartMotorController smc, double expected) throws InterruptedException {
    double kP = Double.NaN;
    for (int i = 0; i < 50; i++) {
      final Object motorController = smc.getMotorController();
      if (motorController instanceof SparkMax sparkMax) {
        kP = sparkMax.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0);
      } else if (motorController instanceof SparkFlex sparkFlex) {
        kP = sparkFlex.configAccessor.closedLoop.getP(ClosedLoopSlot.kSlot0);
      } else {
        final Slot0Configs slot0 = new Slot0Configs();
        if (motorController instanceof TalonFXS talonFXS) {
          talonFXS.getConfigurator().refresh(slot0);
        } else {
          ((TalonFX) motorController).getConfigurator().refresh(slot0);
        }
        kP = slot0.kP;
      }
      if (Math.abs(kP - expected) <= 1e-6 * Math.abs(expected)) {
        return kP;
      }
      Thread.sleep(10);
    }
    return kP;
  }

  @ParameterizedTest(name = "{0}")
  @MethodSource("createControllers")
  void tuningUsesTheConfigsValues(Case testCase) throws InterruptedException {
    final String name = testCase.name();
    final SmartMotorController smc = testCase.create().apply(config(testCase));
    try {
      ((SmartMotorControllerTestSubsystem) ((yams.commands2.config.SmartMotorControllerConfig) smc.getConfig()).getSubsystem()).setSMC(smc);
      final NetworkTable telemetryTable = NetworkTableInstance.getDefault().getTable("LiveTuningTest");
      final NetworkTable tuningTable = NetworkTableInstance.getDefault().getTable("LiveTuningTestTuning");
      smc.setupTelemetry(telemetryTable, tuningTable);
      final NetworkTable tuning = tuningTable.getSubTable("LiveTuningTest " + name);

      // The tuning table shows the config's values.
      assertShown(tuning, "closedloop/feedback/kP", 1.5, name);
      assertShown(tuning, "closedloop/feedback/kI", 0.25, name);
      assertShown(tuning, "closedloop/feedback/kD", 0.125, name);
      assertShown(tuning, "closedloop/feedforward/kS", 0.2, name);
      assertShown(tuning, "closedloop/feedforward/kV", 0.75, name);
      assertShown(tuning, "closedloop/feedforward/kA", 0.05, name);
      switch (testCase.controller()) {
        case TRAPEZOIDAL_PROFILE -> {
          assertShown(tuning, "closedloop/motionprofile/maxVelocity",
              testCase.linear() ? 1.5 : RotationsPerSecond.of(2).in(RPM), name);
          assertShown(tuning, "closedloop/motionprofile/maxAcceleration",
              testCase.linear() ? 3 : RotationsPerSecondPerSecond.of(4).in(RPM.per(Second)), name);
        }
        case EXPONENTIAL_PROFILE -> {
          assertShown(tuning, "closedloop/motionprofile/kV", 1.5, name);
          assertShown(tuning, "closedloop/motionprofile/kA", 0.3, name);
          assertShown(tuning, "closedloop/motionprofile/maxInput", 12, name);
        }
        default -> {
        }
      }

      // Enter new values, one after another, and apply them.
      enter(tuning, "closedloop/feedback/kP", 2.5);
      enter(tuning, "closedloop/feedback/kI", 0.5);
      enter(tuning, "closedloop/feedback/kD", 0.375);
      enter(tuning, "closedloop/feedforward/kS", 0.3);
      enter(tuning, "closedloop/feedforward/kV", 1.25);
      enter(tuning, "closedloop/feedforward/kA", 0.1);
      switch (testCase.controller()) {
        case TRAPEZOIDAL_PROFILE -> {
          enter(tuning, "closedloop/motionprofile/maxVelocity", testCase.linear() ? 2.5 : RotationsPerSecond.of(3).in(RPM));
          enter(tuning, "closedloop/motionprofile/maxAcceleration",
              testCase.linear() ? 5 : RotationsPerSecondPerSecond.of(6).in(RPM.per(Second)));
        }
        case EXPONENTIAL_PROFILE -> {
          enter(tuning, "closedloop/motionprofile/kV", 1.8);
          enter(tuning, "closedloop/motionprofile/kA", 0.45);
          enter(tuning, "closedloop/motionprofile/maxInput", 10);
        }
        default -> {
        }
      }
      // Hold the mechanism where it is while tuning.
      enter(tuning, "closedloop/setpoint/position", testCase.linear() ? 0 : Degrees.of(0).in(Degrees));
      smc.applyTuningValues();

      // The config holds exactly the entered values.
      final var cfg = smc.getConfig();
      final var pid = cfg.getPID(ClosedLoopControllerSlot.SLOT_0).orElseThrow();
      assertEquals(2.5, pid.getP(), kTolerance, name + ": config kP");
      assertEquals(0.5, pid.getI(), kTolerance, name + ": config kI");
      assertEquals(0.375, pid.getD(), kTolerance, name + ": config kD");
      final var feedforward = cfg.getSimpleFeedforward(ClosedLoopControllerSlot.SLOT_0).orElseThrow();
      assertEquals(0.3, feedforward.getKs(), kTolerance, name + ": config kS");
      assertEquals(1.25, feedforward.getKv(), kTolerance, name + ": config kV");
      assertEquals(0.1, feedforward.getKa(), kTolerance, name + ": config kA");
      switch (testCase.controller()) {
        case TRAPEZOIDAL_PROFILE -> {
          final TrapezoidProfile.Constraints constraints = cfg.getTrapezoidProfile().orElseThrow();
          assertEquals(testCase.linear() ? 2.5 : 3, constraints.maxVelocity, 1e-9, name + ": config max velocity");
          assertEquals(testCase.linear() ? 5 : 6, constraints.maxAcceleration, 1e-9, name + ": config max acceleration");
        }
        case EXPONENTIAL_PROFILE -> {
          final ExponentialProfile.Constraints constraints = cfg.getExponentialProfile().orElseThrow();
          assertEquals(1.8, -constraints.A / constraints.B, 1e-9, name + ": config profile kV");
          assertEquals(0.45, 1.0 / constraints.B, 1e-9, name + ": config profile kA");
          assertEquals(10, constraints.maxInput, 1e-9, name + ": config profile max input");
        }
        default -> {
        }
      }

      // The motor controller holds the entered kP in its own units: duty cycle per motor rotation on
      // a SPARK, volts per mechanism rotation on a Talon, and per meter converted to per rotation for
      // a linear mechanism.
      final double rotationsPerGainUnit = testCase.linear() ? 1 / kCircumferenceMeters : 1;
      final double expectedDeviceKp = smc instanceof SparkWrapper
                                      ? 2.5 / (12 * kGearing.getMechanismToRotorRatio() * rotationsPerGainUnit)
                                      : 2.5 / rotationsPerGainUnit;
      final double deviceKp = deviceKp(smc, expectedDeviceKp);
      assertTrue(Math.abs(deviceKp - expectedDeviceKp) <= 1e-6 * expectedDeviceKp,
          name + ": device kP expected " + expectedDeviceKp + " but was " + deviceKp);
    } finally {
      closeSmc(smc);
    }
  }
}
