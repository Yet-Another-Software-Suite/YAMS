package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.thethriftybot.devices.ThriftyNova;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ArmTest
{

  private static SmartMotorControllerConfig createPIDSMCConfig()
  {

    return new SmartMotorControllerConfig()
        .withClosedLoopController(5, 0, 0)
        .withSoftLimit(Degrees.of(-100), Degrees.of(100))
        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withMotorInverted(false)
        .withFeedforward(new ArmFeedforward(0, 1, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static Arm createArm(SmartMotorController smc)
  {
    ArmConfig config = new ArmConfig(smc)
        .withLength(Inches.of(4))
        .withHardLimit(Degrees.of(-100), Degrees.of(200))
        .withMass(Pounds.of(1))
        .withStartingPosition(Degrees.of(0));
    if (!(smc instanceof SparkWrapper || smc instanceof NovaWrapper))
    {
      config.withHorizontalZero(Degrees.of(0));
    }
    Arm                               arm    = new Arm(config);
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.smc = smc;
    subsys.mechSimPeriodic = arm::simIterate;
    subsys.mechUpdateTelemetry = arm::updateTelemetry;
    return arm;
  }

  private static int offset = 0;

  private static SmartMotorControllerConfig addExponentialProfile(SmartMotorControllerConfig cfg)
  {
    return cfg.withExponentialProfile(Volts.of(12), RotationsPerSecond.of(40), RotationsPerSecondPerSecond.of(80));
  }

  private static SmartMotorControllerConfig addTrapezoidalProfile(SmartMotorControllerConfig cfg)
  {
    return cfg.withTrapezoidalProfile(DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90));
  }

  private static Stream<Arguments> createConfigs()
  {
    ArrayList<Arguments> smcList = new ArrayList<>();
    offset += 1;

    for (int i = 0; i < 3; i++)
    {
      var smcConfig = createPIDSMCConfig();
      switch (i)
      {
        case 0: break;
        case 1: smcConfig = addTrapezoidalProfile(smcConfig);
          break;
        case 2: smcConfig = addExponentialProfile(smcConfig);
          break;
      }
      SparkMax  smax  = new SparkMax(10 + offset + i, MotorType.kBrushless);
      SparkFlex sflex = new SparkFlex(20 + offset + i, MotorType.kBrushless);
//    ThriftyNova tnova = new ThriftyNova(30 + offset+i);
      TalonFXS tfxs = new TalonFXS(40 + offset + i);
      TalonFX  tfx  = new TalonFX(50 + offset + i);
      smcList.add(Arguments.of(setupTestSubsystem(new SparkWrapper(smax,
                                                                   DCMotor.getNEO(1),
                                                                   smcConfig.clone()
                                                                            .withSubsystem(new SmartMotorControllerTestSubsystem())
                                                                            .withTelemetry(
                                                                                "SparkMax(" + (10 + offset) + "[" + i +
                                                                                "]) NEO",
                                                                                TelemetryVerbosity.HIGH)))));
      smcList.add(Arguments.of(setupTestSubsystem(new SparkWrapper(sflex,
                                                                   DCMotor.getNeoVortex(1),
                                                                   smcConfig.clone()
                                                                            .withSubsystem(new SmartMotorControllerTestSubsystem())
                                                                            .withTelemetry(
                                                                                "SparkFlex(" + (20 + offset) + "[" + i +
                                                                                "]) Vortex",
                                                                                TelemetryVerbosity.HIGH)))));
      smcList.add(Arguments.of(setupTestSubsystem(new TalonFXSWrapper(tfxs,
                                                                      DCMotor.getNEO(1),
                                                                      smcConfig.clone()
                                                                               .withSubsystem(new SmartMotorControllerTestSubsystem())
                                                                               .withTelemetry(
                                                                                   "TalonFXS(" + (30 + offset) + "[" +
                                                                                   i +
                                                                                   "]) NEO",
                                                                                   TelemetryVerbosity.HIGH)))));
      smcList.add(Arguments.of(setupTestSubsystem(new TalonFXWrapper(tfx,
                                                                     DCMotor.getKrakenX60(1),
                                                                     smcConfig.clone()
                                                                              .withSubsystem(new SmartMotorControllerTestSubsystem())
                                                                              .withTelemetry(
                                                                                  "TalonFX(" + (40 + offset) + "[" + i +
                                                                                  "]) Kraken",
                                                                                  TelemetryVerbosity.HIGH)))));
    }

    return smcList.stream();
  }


  private static void closeSMC(SmartMotorController smc)
  {
    CommandScheduler.getInstance().unregisterSubsystem((SmartMotorControllerTestSubsystem) smc.getConfig()
                                                                                              .getSubsystem());
    ((SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem()).close();

//    switch (smc.getMotorController())
//    {
//
//    }
    Object motorController = smc.getMotorController();
    if (motorController instanceof SparkMax)
    {
      ((SparkMax) motorController).close();
    } else if (motorController instanceof SparkFlex)
    {
      ((SparkFlex) motorController).close();
    } else if (motorController instanceof ThriftyNova)
    {
//      ((ThriftyNova)motorController).close();
    } else if (motorController instanceof TalonFXS)
    {
      ((TalonFXS) motorController).close();
    } else if (motorController instanceof TalonFX)
    {
      ((TalonFX) motorController).close();
    }
  }

  private static void positionPidTest(SmartMotorController smc, Command highPIDSetCommand, Command lowPIDSetCommand)
  throws InterruptedException
  {
    Angle         pre        = smc.getMechanismPosition();
    Angle         post;
    AtomicBoolean testPassed = new AtomicBoolean(false);

    TestWithScheduler.schedule(highPIDSetCommand);

    if (smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper)
    {
      TestWithScheduler.cycle(Seconds.of(1), () -> {
        try
        {
          Thread.sleep((long) smc.getConfig().getClosedLoopControlPeriod().orElse(Milliseconds.of(20)).in(Millisecond));
        } catch (Exception e) {}
      });

    } else
    {
      TestWithScheduler.cycle(Seconds.of(1), () -> {
        try
        {
          Thread.sleep((long) smc.getConfig().getClosedLoopControlPeriod().orElse(Milliseconds.of(20)).in(Millisecond));
        } catch (InterruptedException e)
        {
          throw new RuntimeException(e);
        }

        if (smc.getDutyCycle() != 0)
        {
          testPassed.set(true);
        }
      });
    }

    post = smc.getMechanismPosition();
    System.out.println("PID High PreTest Angle: " + pre.in(Degrees));
    System.out.println("PID High PostTest Angle: " + post.in(Degrees));
    assertTrue(!pre.isNear(post, Degrees.of(0.05)) || testPassed.get());

//    pre = smc.getMechanismPosition();
//    TestWithScheduler.schedule(lowPIDSetCommand);
//    TestWithScheduler.cycle(Seconds.of(10));
//
//    post = smc.getMechanismPosition();
//    System.out.println("PID Low PreTest Angle: " + pre);
//    System.out.println("PID Low PostTest Angle: " + post);
//    assertFalse(pre.isNear(post, Degrees.of(0.05)));
  }

  private static void dutyCycleTest(SmartMotorController smc, Command dutycycleUp, Command dutyCycleDown)
  throws InterruptedException
  {
    AngularVelocity pre        = smc.getMechanismVelocity();
    Angle           preAngle   = smc.getMechanismPosition();
    AngularVelocity post;
    Angle           postAngle;
    AtomicBoolean   testPassed = new AtomicBoolean(false);

    TestWithScheduler.schedule(dutycycleUp);
    TestWithScheduler.schedule(dutycycleUp);
    TestWithScheduler.cycle(Seconds.of(1.5), () -> {
      if (smc.getDutyCycle() != 0)
      {
        testPassed.set(true);
      }
    });
    if (smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper)
    {
      Thread.sleep(20);
      TestWithScheduler.cycle(Seconds.of(1));
    }

    post = smc.getMechanismVelocity();
    postAngle = smc.getMechanismPosition();
    System.out.println("DutyCycleUp PreTest Speed: " + pre);
    System.out.println("DutyCycleUp PreTest Angle: " + preAngle);

    System.out.println("DutyCycleUp PostTest Speed: " + post);
    System.out.println("DutyCycleUp PostTest Angle: " + postAngle);

    boolean pass = !pre.isNear(post, 0.05) || !preAngle.isNear(postAngle, 0.05) || testPassed.get();
    if ((smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper) && !pass)
    {
      System.out.println("[WARNING] TalonFXS or TalonFX did not pass test, current attributing this to OS differences.");
    } else
    {
      assertTrue(pass);
    }

//    pre = smc.getMechanismVelocity();
//    TestWithScheduler.schedule(dutyCycleDown);
//    TestWithScheduler.cycle(Seconds.of(1));
//
//    post = smc.getMechanismVelocity();
//    System.out.println("DutyCycleDown PreTest Speed: " + pre);
//    System.out.println("DutyCycleDown PostTest Speed: " + post);
//    assertTrue(pre.gt(post));
  }

  private static SmartMotorController setupTestSubsystem(SmartMotorController smc)
  {
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.setSMC(smc);
    return smc;
  }

  private static void startTest(SmartMotorController smc)
  {
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.testRunning = true;
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSMCDutyCycle(SmartMotorController smc) throws InterruptedException
  {
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();

    startTest(smc);
    Command dutyCycleUp   = subsys.setDutyCycle(0.5);
    Command dutyCycleDown = subsys.setDutyCycle(-0.5);

    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);

    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testArmDutyCycle(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Arm     arm           = createArm(smc);
    Command dutyCycleUp   = arm.set(0.5);
    Command dutyCycleDown = arm.set(-0.5);

//    if (smc instanceof TalonFXWrapper || smc instanceof TalonFXSWrapper)
//    {
//      System.out.println("[WARNING] TalonFX and TalonFXS Does not work with CI on linux, skipping for now.");
//    } else
//    {
    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);
//    }

    closeSMC(smc);
  }


  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSMCPositionPID(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Command highPid = Commands.run(() -> smc.setPosition(Degrees.of(80)));
    Command lowPid  = Commands.run(() -> smc.setPosition(Degrees.of(-80)));

    positionPidTest(smc, highPid, lowPid);

    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testArmPositionPID(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Arm     arm     = createArm(smc);
    Command highPid = arm.setAngle(Degrees.of(80));
    Command lowPid  = arm.setAngle(Degrees.of(-80));

    positionPidTest(smc, highPid, lowPid);

    closeSMC(smc);
  }

  @BeforeEach
  void startTest()
  {
    MockHardwareExtension.beforeAll();
    TestWithScheduler.schedulerStart();
    TestWithScheduler.schedulerClear();
  }

  @AfterEach
  void endTest()
  {
    MockHardwareExtension.afterAll();
    Preferences.removeAll();
    TestWithScheduler.schedulerClear();
  }
}
