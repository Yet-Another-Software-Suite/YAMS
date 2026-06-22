// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.ArrayList;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import yams.exceptions.ArmConfigurationException;
import yams.exceptions.ElevatorConfigurationException;
import yams.exceptions.PivotConfigurationException;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.Elevator;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

/**
 * Verifies that:
 *
 * <ol>
 *   <li>min()/max() Triggers fire in the correct direction for Pivot, Arm, and Elevator. Starting
 *       positions are configured in SmartMotorControllerConfig so the mechanism sim is seeded via
 *       the normal construction path — not via setEncoderPosition.
 *   <li>A configured starting position is reflected in getMechanismPosition() after setupSimulation
 *       + one scheduler cycle.
 *   <li>Constructors throw when the starting position exceeds the mechanism's hard limits.
 * </ol>
 */
public class MechanismLimitTest
{
  /**
   * Monotonically-increasing counter for unique CAN device IDs across all @MethodSource factory
   * calls. SparkMax and SparkFlex share the REV device registry, so their ID ranges must not
   * overlap with each other or with IDs used by other test classes. Other test classes (ArmTest,
   * ElevatorTest, PivotTest, ShooterTest) use SparkMax 11–16, SparkFlex 21–26, TalonFXS 41–46,
   * TalonFX 51–56 — MechanismLimitTest must stay outside those ranges. With at most 7 factory
   * calls the ranges are:
   *
   * <pre>
   *   REV:  SparkMax  27+offset → 28–34  SparkFlex 47+offset → 48–54
   *   CTRE: TalonFXS   1+offset →  2–8   TalonFX   15+offset → 16–22
   * </pre>
   *
   * Each factory also creates a second "at-limit" batch:
   *
   * <pre>
   *   REV:  SparkMax  34+offset → 35–41  SparkFlex 54+offset → 55–61
   *   CTRE: TalonFXS   9+offset → 10–16  TalonFX   23+offset → 24–30
   * </pre>
   *
   * Exception tests use SparkMax 42–47 (EXCEPT_BASE = 42). All IDs are ≤ 61 and no REV range
   * overlaps another REV range.
   */
  private static int offset = 0;

  // Fixed IDs for the single-motor exception tests — in the gap between SparkMax true (35–41)
  // and SparkFlex false (48–54), so they never conflict with any parameterized REV device.
  private static final int EXCEPT_BASE = 42;

  // ──────────────────────────────────────────────
  // SMC config base factories
  // ──────────────────────────────────────────────

  private static SmartMotorControllerConfig pivotBase()
  {
    return new SmartMotorControllerConfig()
        .withClosedLoopController(4, 0, 0)
        .withSoftLimits(Degrees.of(-100), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4, 5)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(1, 0, 0, 0.02))
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withMomentOfInertia(Inches.of(4), Pounds.of(1));
  }

  private static SmartMotorControllerConfig armBase()
  {
    return new SmartMotorControllerConfig()
        .withClosedLoopController(5, 0, 0)
        .withSoftLimits(Degrees.of(-100), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new ArmFeedforward(0, 1, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static SmartMotorControllerConfig elevatorBase()
  {
    return new SmartMotorControllerConfig()
        .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
        .withClosedLoopController(4, 0, 0)
        .withSoftLimits(Meters.of(0), Meters.of(5))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  // ──────────────────────────────────────────────
  // Mechanism factories
  //
  // createPivot explicitly calls setupSimulation because Pivot's constructor
  // does not call it (unlike Arm and Elevator which do).
  // ──────────────────────────────────────────────

  private static Pivot createPivot(SmartMotorController smc)
  {
    PivotConfig config = new PivotConfig(smc).withHardLimits(Degrees.of(-100), Degrees.of(150));
    Pivot     pivot  = new Pivot(config);
    smc.setupSimulation();
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.mechSimPeriodic = pivot::simIterate;
    subsys.mechUpdateTelemetry = pivot::updateTelemetry;
    return pivot;
  }

  private static Arm createArm(SmartMotorController smc)
  {
    // Hard limits are 5° wider at the bottom than the soft limits so that a
    // "true at min" starting position (-105°) can clear the constructor's
    // bounds check while still being below the -100° soft limit.
    ArmConfig config = new ArmConfig(smc)
        .withLength(Inches.of(4))
        .withHardLimits(Degrees.of(-110), Degrees.of(200))
        .withMass(Pounds.of(1));
    Arm arm = new Arm(config);
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.mechSimPeriodic = arm::simIterate;
    subsys.mechUpdateTelemetry = arm::updateTelemetry;
    return arm;
  }

  /**
   * Creates an Elevator from {@code smc}. Starting height comes from the starting position
   * already configured in the SMC config by the factory method — do NOT call
   * {@code withStartingHeight} here, or it would overwrite the factory-supplied seed.
   * Hard limits extend 0.1 m below 0 so that a "true at min" starting position
   * (-0.01 m) clears the bounds check while remaining below the 0 m soft limit.
   */
  private static Elevator createElevator(SmartMotorController smc)
  {
    ElevatorConfig config = new ElevatorConfig(smc)
        .withHardLimits(Meters.of(-0.1), Meters.of(6))
        .withMass(Pounds.of(16));
    Elevator elevator = new Elevator(config);
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.mechSimPeriodic = elevator::simIterate;
    subsys.mechUpdateTelemetry = elevator::updateTelemetry;
    return elevator;
  }

  // ──────────────────────────────────────────────
  // SMC builder helpers
  // ──────────────────────────────────────────────

  private static SmartMotorController makePivotSparkMax(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkMax(id, MotorType.kBrushless), DCMotor.getNEO(1),
        pivotBase().withStartingPosition(startPos)
                   .withSubsystem(new SmartMotorControllerTestSubsystem())
                   .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makePivotSparkFlex(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkFlex(id, MotorType.kBrushless), DCMotor.getNeoVortex(1),
        pivotBase().withStartingPosition(startPos)
                   .withSubsystem(new SmartMotorControllerTestSubsystem())
                   .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makePivotTalonFXS(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new TalonFXSWrapper(new TalonFXS(id), DCMotor.getNEO(1),
        pivotBase().withStartingPosition(startPos)
                   .withSubsystem(new SmartMotorControllerTestSubsystem())
                   .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makePivotTalonFX(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new TalonFXWrapper(new TalonFX(id), DCMotor.getKrakenX60(1),
        pivotBase().withStartingPosition(startPos)
                   .withSubsystem(new SmartMotorControllerTestSubsystem())
                   .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeArmSparkMax(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkMax(id, MotorType.kBrushless), DCMotor.getNEO(1),
        armBase().withStartingPosition(startPos)
                 .withSubsystem(new SmartMotorControllerTestSubsystem())
                 .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeArmSparkFlex(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkFlex(id, MotorType.kBrushless), DCMotor.getNeoVortex(1),
        armBase().withStartingPosition(startPos)
                 .withSubsystem(new SmartMotorControllerTestSubsystem())
                 .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeArmTalonFXS(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new TalonFXSWrapper(new TalonFXS(id), DCMotor.getNEO(1),
        armBase().withStartingPosition(startPos)
                 .withSubsystem(new SmartMotorControllerTestSubsystem())
                 .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeArmTalonFX(int id, Angle startPos, String label)
  {
    return setupTestSubsystem(new TalonFXWrapper(new TalonFX(id), DCMotor.getKrakenX60(1),
        armBase().withStartingPosition(startPos)
                 .withSubsystem(new SmartMotorControllerTestSubsystem())
                 .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeElevSparkMax(int id, Distance startHeight, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkMax(id, MotorType.kBrushless), DCMotor.getNEO(1),
        elevatorBase().withStartingPosition(startHeight)
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeElevSparkFlex(int id, Distance startHeight, String label)
  {
    return setupTestSubsystem(new SparkWrapper(new SparkFlex(id, MotorType.kBrushless), DCMotor.getNeoVortex(1),
        elevatorBase().withStartingPosition(startHeight)
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeElevTalonFXS(int id, Distance startHeight, String label)
  {
    return setupTestSubsystem(new TalonFXSWrapper(new TalonFXS(id), DCMotor.getNEO(1),
        elevatorBase().withStartingPosition(startHeight)
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  private static SmartMotorController makeElevTalonFX(int id, Distance startHeight, String label)
  {
    return setupTestSubsystem(new TalonFXWrapper(new TalonFX(id), DCMotor.getKrakenX60(1),
        elevatorBase().withStartingPosition(startHeight)
                      .withSubsystem(new SmartMotorControllerTestSubsystem())
                      .withTelemetry(label, TelemetryVerbosity.HIGH)));
  }

  // ──────────────────────────────────────────────
  // Argument stream factories
  //
  // Each factory creates 8 Arguments: 4 motor types at a "false" starting
  // position, then 4 at a "true" (at-limit) starting position. The boolean
  // second parameter tells the test what the trigger should return.
  //
  // Device ID ranges (7 factory calls max, offset 1–7):
  //   REV  SparkMax  : false → 27+o (28-34)  true → 34+o (35-41)
  //   REV  SparkFlex : false → 47+o (48-54)  true → 54+o (55-61)
  //   CTRE TalonFXS  : false →  1+o  (2-8)   true →  9+o (10-16)
  //   CTRE TalonFX   : false → 15+o (16-22)  true → 23+o (24-30)
  //   REV  EXCEPT     SparkMax 42–47 (EXCEPT_BASE)
  // All ≤ 61; no REV range overlaps another REV range; none overlap other test classes.
  // ──────────────────────────────────────────────

  private static Stream<Arguments> pivotMinArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Degrees.of(0), false, ""},
        {Degrees.of(-100), true, "-atMin"},
    })
    {
      Angle   pos      = (Angle) row[0];
      boolean expected = (boolean) row[1];
      String  sfx      = (String) row[2];
      int     base     = expected ? 34 : 27;
      int     fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makePivotSparkMax(base + offset, pos, "SparkMax(pMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotSparkFlex(fbase + offset, pos, "SparkFlex(pMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotTalonFXS(base + offset, pos, "TalonFXS(pMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotTalonFX(fbase + offset, pos, "TalonFX(pMin" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> pivotMaxArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Degrees.of(0), false, ""},
        // 101° is 1° past the 100° soft limit; DCMotorSim has no gravity so the
        // position stays near 101° after the physics step, well above 100°.
        {Degrees.of(101), true, "-atMax"},
    })
    {
      Angle   pos      = (Angle) row[0];
      boolean expected = (boolean) row[1];
      String  sfx      = (String) row[2];
      int     base     = expected ? 34 : 27;
      int     fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makePivotSparkMax(base + offset, pos, "SparkMax(pMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotSparkFlex(fbase + offset, pos, "SparkFlex(pMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotTalonFXS(base + offset, pos, "TalonFXS(pMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makePivotTalonFX(fbase + offset, pos, "TalonFX(pMax" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> armMinArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Degrees.of(0), false, ""},
        // -105° is 5° past the -100° soft limit so gravity's restoring force
        // (which pushes the arm back toward 0°) cannot drift it above -100° in 20 ms.
        {Degrees.of(-105), true, "-atMin"},
    })
    {
      Angle   pos      = (Angle) row[0];
      boolean expected = (boolean) row[1];
      String  sfx      = (String) row[2];
      int     base     = expected ? 34 : 27;
      int     fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makeArmSparkMax(base + offset, pos, "SparkMax(aMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmSparkFlex(fbase + offset, pos, "SparkFlex(aMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmTalonFXS(base + offset, pos, "TalonFXS(aMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmTalonFX(fbase + offset, pos, "TalonFX(aMin" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> armMaxArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Degrees.of(0), false, ""},
        // 101° is 1° past the 100° soft limit; at 101° gravity pushes the arm further
        // from horizontal (past vertical), so the position stays above 100° after one
        // physics step.
        {Degrees.of(101), true, "-atMax"},
    })
    {
      Angle   pos      = (Angle) row[0];
      boolean expected = (boolean) row[1];
      String  sfx      = (String) row[2];
      int     base     = expected ? 34 : 27;
      int     fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makeArmSparkMax(base + offset, pos, "SparkMax(aMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmSparkFlex(fbase + offset, pos, "SparkFlex(aMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmTalonFXS(base + offset, pos, "TalonFXS(aMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeArmTalonFX(fbase + offset, pos, "TalonFX(aMax" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> elevatorMinArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Meters.of(1.5), false, ""},
        // -0.01 m is below the 0 m soft limit; avoids float-point epsilon where
        // exactly 0 m reads back as ~8e-8 m (> 0) causing lte(0) = false.
        {Meters.of(-0.01), true, "-atMin"},
    })
    {
      Distance pos      = (Distance) row[0];
      boolean  expected = (boolean) row[1];
      String   sfx      = (String) row[2];
      int      base     = expected ? 34 : 27;
      int      fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makeElevSparkMax(base + offset, pos, "SparkMax(eMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevSparkFlex(fbase + offset, pos, "SparkFlex(eMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevTalonFXS(base + offset, pos, "TalonFXS(eMin" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevTalonFX(fbase + offset, pos, "TalonFX(eMin" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> elevatorMaxArgs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    for (Object[] row : new Object[][]{
        {Meters.of(1.5), false, ""},
        // 5.5 m > 5 m soft limit; gravity drops the elevator slightly in one step
        // but it remains well above 5 m.
        {Meters.of(5.5), true, "-atMax"},
    })
    {
      Distance pos      = (Distance) row[0];
      boolean  expected = (boolean) row[1];
      String   sfx      = (String) row[2];
      int      base     = expected ? 34 : 27;
      int      fbase    = expected ? 54 : 47;
      args.add(Arguments.of(makeElevSparkMax(base + offset, pos, "SparkMax(eMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevSparkFlex(fbase + offset, pos, "SparkFlex(eMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevTalonFXS(base + offset, pos, "TalonFXS(eMax" + sfx + ")[" + offset + "]"), expected));
      args.add(Arguments.of(makeElevTalonFX(fbase + offset, pos, "TalonFX(eMax" + sfx + ")[" + offset + "]"), expected));
    }
    return args.stream();
  }

  private static Stream<Arguments> createStartingPosConfigs()
  {
    offset++;
    var args = new ArrayList<Arguments>();
    args.add(Arguments.of(makePivotSparkMax(27 + offset, Degrees.of(45), "SparkMax(spos)[" + offset + "]")));
    args.add(Arguments.of(makePivotSparkFlex(47 + offset, Degrees.of(45), "SparkFlex(spos)[" + offset + "]")));
    args.add(Arguments.of(makePivotTalonFXS(1 + offset, Degrees.of(45), "TalonFXS(spos)[" + offset + "]")));
    args.add(Arguments.of(makePivotTalonFX(15 + offset, Degrees.of(45), "TalonFX(spos)[" + offset + "]")));
    return args.stream();
  }

  // ──────────────────────────────────────────────
  // Helpers
  // ──────────────────────────────────────────────

  private static SmartMotorController setupTestSubsystem(SmartMotorController smc)
  {
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).setSMC(smc);
    return smc;
  }

  private static void startTest(SmartMotorController smc)
  {
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).testRunning = true;
  }

  private static void closeSMC(SmartMotorController smc)
  {
    CommandScheduler.getInstance()
                    .unregisterSubsystem((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem());
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).close();
    Object motor = smc.getMotorController();
    if (motor instanceof SparkMax)
    {
      ((SparkMax) motor).close();
    } else if (motor instanceof SparkFlex)
    {
      ((SparkFlex) motor).close();
    } else if (motor instanceof TalonFXS)
    {
      ((TalonFXS) motor).close();
    } else if (motor instanceof TalonFX)
    {
      ((TalonFX) motor).close();
    }
  }

  // ──────────────────────────────────────────────
  // Trigger direction tests
  //
  // These verify that min()/max() Triggers fire correctly based on a starting
  // position that was embedded in the SMC config at construction time. The
  // mechanism sim is seeded via the normal construction path (setupSimulation is
  // called inside createPivot/createArm/createElevator). One scheduler cycle runs
  // simIterate and SimHooks.stepTimingAsync so Phoenix 6 status signals reflect
  // the seeded position before the trigger is read.
  // ──────────────────────────────────────────────

  @ParameterizedTest
  @MethodSource("pivotMinArgs")
  void testPivotMinTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Pivot pivot = createPivot(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, pivot.min().getAsBoolean(),
                 "pivot.min() at " + smc.getMechanismPosition() + " vs soft lower limit (-100°)");
    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("pivotMaxArgs")
  void testPivotMaxTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Pivot pivot = createPivot(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, pivot.max().getAsBoolean(),
                 "pivot.max() at " + smc.getMechanismPosition() + " vs soft upper limit (100°)");
    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("armMinArgs")
  void testArmMinTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Arm arm = createArm(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, arm.min().getAsBoolean(),
                 "arm.min() at " + smc.getMechanismPosition() + " vs soft lower limit (-100°)");
    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("armMaxArgs")
  void testArmMaxTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Arm arm = createArm(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, arm.max().getAsBoolean(),
                 "arm.max() at " + smc.getMechanismPosition() + " vs soft upper limit (100°)");
    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("elevatorMinArgs")
  void testElevatorMinTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Elevator elevator = createElevator(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, elevator.min().getAsBoolean(),
                 "elevator.min() at " + smc.getMechanismPosition() + " vs soft lower limit (0 m)");
    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("elevatorMaxArgs")
  void testElevatorMaxTrigger(SmartMotorController smc, boolean expectedTrigger) throws InterruptedException
  {
    Elevator elevator = createElevator(smc);
    startTest(smc);
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertEquals(expectedTrigger, elevator.max().getAsBoolean(),
                 "elevator.max() at " + smc.getMechanismPosition() + " vs soft upper limit (5 m)");
    closeSMC(smc);
  }

  // ──────────────────────────────────────────────
  // Starting position round-trip test
  //
  // Verifies that the starting position configured in SmartMotorControllerConfig
  // is reflected in getMechanismPosition() after setupSimulation() and one cycle.
  // Pivot is used because its constructor does not call setupSimulation() itself,
  // making this an explicit test of that path.
  // ──────────────────────────────────────────────

  @ParameterizedTest
  @MethodSource("createStartingPosConfigs")
  void testStartingPositionAppliedAfterSetup(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    smc.setupSimulation();
    TestWithScheduler.cycle(Milliseconds.of(20));
    assertTrue(smc.getMechanismPosition().isNear(Degrees.of(45), Degrees.of(2)),
               "Expected starting position ~45° but got " + smc.getMechanismPosition());
    closeSMC(smc);
  }

  // ──────────────────────────────────────────────
  // Starting position out-of-bounds exception tests
  //
  // Fixed device IDs 50-55 (REV namespace) for the 6 single-SparkMax tests.
  // ──────────────────────────────────────────────

  @Test
  void testPivotThrowsWhenStartingPositionBelowLowerLimit()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE, MotorType.kBrushless), DCMotor.getNEO(1),
        pivotBase().withStartingPosition(Degrees.of(-200)).withSubsystem(new SmartMotorControllerTestSubsystem())));
    try
    {
      assertThrows(PivotConfigurationException.class, () ->
          new Pivot(new PivotConfig(motor).withHardLimits(Degrees.of(-100), Degrees.of(150))));
    } finally
    {
      closeSMC(motor);
    }
  }

  @Test
  void testPivotThrowsWhenStartingPositionAboveUpperLimit()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE + 1, MotorType.kBrushless), DCMotor.getNEO(1),
        pivotBase().withStartingPosition(Degrees.of(200)).withSubsystem(new SmartMotorControllerTestSubsystem())));
    try
    {
      assertThrows(PivotConfigurationException.class, () ->
          new Pivot(new PivotConfig(motor).withHardLimits(Degrees.of(-100), Degrees.of(150))));
    } finally
    {
      closeSMC(motor);
    }
  }

  @Test
  void testArmThrowsWhenStartingPositionBelowLowerLimit()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE + 2, MotorType.kBrushless), DCMotor.getNEO(1),
        armBase().withStartingPosition(Degrees.of(-200)).withSubsystem(new SmartMotorControllerTestSubsystem())));
    try
    {
      assertThrows(ArmConfigurationException.class, () ->
          new Arm(new ArmConfig(motor).withLength(Inches.of(4))
                                       .withHardLimits(Degrees.of(-100), Degrees.of(200))
                                       .withMass(Pounds.of(1))));
    } finally
    {
      closeSMC(motor);
    }
  }

  @Test
  void testArmThrowsWhenStartingPositionAboveUpperLimit()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE + 3, MotorType.kBrushless), DCMotor.getNEO(1),
        armBase().withStartingPosition(Degrees.of(300)).withSubsystem(new SmartMotorControllerTestSubsystem())));
    try
    {
      assertThrows(ArmConfigurationException.class, () ->
          new Arm(new ArmConfig(motor).withLength(Inches.of(4))
                                       .withHardLimits(Degrees.of(-100), Degrees.of(200))
                                       .withMass(Pounds.of(1))));
    } finally
    {
      closeSMC(motor);
    }
  }

  @Test
  void testElevatorThrowsWhenStartingHeightBelowMinimum()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE + 4, MotorType.kBrushless), DCMotor.getNEO(1),
        elevatorBase().withSubsystem(new SmartMotorControllerTestSubsystem())
                .withStartingPosition(Meters.of(-1))));
    try
    {
      assertThrows(ElevatorConfigurationException.class, () ->
          new Elevator(new ElevatorConfig(motor)
                           .withHardLimits(Meters.of(0), Meters.of(6))
                           .withMass(Pounds.of(16))));
    } finally
    {
      closeSMC(motor);
    }
  }

  @Test
  void testElevatorThrowsWhenStartingHeightAboveMaximum()
  {
    SmartMotorController motor = setupTestSubsystem(new SparkWrapper(
        new SparkMax(EXCEPT_BASE + 5, MotorType.kBrushless), DCMotor.getNEO(1),
        elevatorBase().withSubsystem(new SmartMotorControllerTestSubsystem())
                .withStartingPosition(Meters.of(7))));
    try
    {
      assertThrows(ElevatorConfigurationException.class, () ->
          new Elevator(new ElevatorConfig(motor)
                           .withHardLimits(Meters.of(0), Meters.of(6))
                           .withMass(Pounds.of(16))));
    } finally
    {
      closeSMC(motor);
    }
  }

  // ──────────────────────────────────────────────
  // Lifecycle
  // ──────────────────────────────────────────────

  @BeforeEach
  void beforeTest()
  {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void afterTest()
  {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
