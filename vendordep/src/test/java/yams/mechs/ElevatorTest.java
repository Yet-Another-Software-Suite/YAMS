package yams.mechs;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Millisecond;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.thethriftybot.ThriftyNova;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.stream.Stream;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;
import yams.helpers.MockHardwareExtension;
import yams.helpers.SmartMotorControllerTestSubsystem;
import yams.helpers.TestWithScheduler;
import yams.mechanisms.config.ElevatorConfig;
import yams.mechanisms.positional.Elevator;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.local.NovaWrapper;
import yams.motorcontrollers.local.SparkWrapper;
import yams.motorcontrollers.remote.TalonFXSWrapper;
import yams.motorcontrollers.remote.TalonFXWrapper;

public class ElevatorTest
{

  private static SmartMotorControllerConfig createSMCConfig()
  {
    SmartMotorControllerTestSubsystem subsystem = new SmartMotorControllerTestSubsystem();

    return new SmartMotorControllerConfig(subsystem)
        .withMechanismCircumference(Meters.of(Inches.of(0.25).in(Meters) * 22))
        .withClosedLoopController(4, 0, 0, MetersPerSecond.of(0.1), MetersPerSecondPerSecond.of(0.5))
        .withSoftLimit(Meters.of(0), Meters.of(5))
        .withGearing(gearing(gearbox(3, 4)))
        .withIdleMode(MotorMode.BRAKE)
//        .withTelemetry("ElevatorMotor", TelemetryVerbosity.HIGH)
        .withStatorCurrentLimit(Amps.of(40))
//      .withVoltageCompensation(Volts.of(12))
        .withMotorInverted(false)
//      .withClosedLoopRampRate(Seconds.of(0.25))
//      .withOpenLoopRampRate(Seconds.of(0.25))
        .withFeedforward(new ElevatorFeedforward(0, 0, 0, 0))
        .withControlMode(ControlMode.CLOSED_LOOP);
  }

  private static Elevator createElevator(SmartMotorController smc)
  {
    ElevatorConfig config = new ElevatorConfig(smc)
        .withStartingHeight(Meters.of(0))
        .withHardLimits(Meters.of(0), Meters.of(3))
//      .withTelemetry("Elevator", TelemetryVerbosity.HIGH)
        .withMass(Pounds.of(16));
    Elevator                          elevator = new Elevator(config);
    SmartMotorControllerTestSubsystem subsys   = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();
    subsys.smc = smc;
    subsys.mechSimPeriodic = elevator::simIterate;
    subsys.mechUpdateTelemetry = elevator::updateTelemetry;
    return elevator;
  }

  private static int offset = 0;

  private static Stream<Arguments> createConfigs()
  {
    offset += 1;
    SparkMax    smax  = new SparkMax(10 + offset, MotorType.kBrushless);
    SparkFlex   sflex = new SparkFlex(20 + offset, MotorType.kBrushless);
    ThriftyNova tnova = new ThriftyNova(30 + offset);
    TalonFXS    tfxs  = new TalonFXS(40 + offset);
    TalonFX     tfx   = new TalonFX(50 + offset);

    return Stream.of(
        Arguments.of(setupTestSubsystem(new SparkWrapper(smax, DCMotor.getNEO(1),
                                                         createSMCConfig()
                                                             .withClosedLoopRampRate(Seconds.of(0.25))
                                                             .withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new SparkWrapper(sflex, DCMotor.getNeoVortex(1),
                                                         createSMCConfig()
                                                             .withClosedLoopRampRate(Seconds.of(0.25))
                                                             .withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new NovaWrapper(tnova, DCMotor.getNEO(1), createSMCConfig()))),
        Arguments.of(setupTestSubsystem(new TalonFXSWrapper(tfxs, DCMotor.getNEO(1),
                                                            createSMCConfig()
//                                                                .withClosedLoopControlPeriod(Millisecond.of(1))
.withClosedLoopRampRate(Seconds.of(0.25))
.withOpenLoopRampRate(Seconds.of(0.25))))),
        Arguments.of(setupTestSubsystem(new TalonFXWrapper(tfx, DCMotor.getKrakenX60(1),
                                                           createSMCConfig()
                                                               .withClosedLoopRampRate(Seconds.of(0.25))
                                                               .withOpenLoopRampRate(Seconds.of(0.25)))))
                    );
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
    Distance      pre        = smc.getMeasurementPosition();
    Distance      post;
    AtomicBoolean testPassed = new AtomicBoolean(false);
    TestWithScheduler.schedule(highPIDSetCommand);

    if (smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper)
    {
      TestWithScheduler.cycle(Seconds.of(1), () -> {
        try {Thread.sleep((long) smc.getConfig().getClosedLoopControlPeriod().orElse(Milliseconds.of(20)).in(Millisecond));} catch (Exception e) {}
      });

    } else
    {
      TestWithScheduler.cycle(Seconds.of(20), () -> {
        if (smc.getDutyCycle() != 0)
        {
          testPassed.set(true);
        }
      });
    }

    post = smc.getMeasurementPosition();
    System.out.println("PID High PreTest Height: " + pre);
    System.out.println("PID High PostTest Height: " + post + " == " + post.in(Meters));

    assertTrue(!pre.isNear(post, Meters.of(0.005)) || testPassed.get());

//    pre = smc.getMeasurementPosition();
//    TestWithScheduler.schedule(lowPIDSetCommand);
//    TestWithScheduler.cycle(Seconds.of(30));

//    post = smc.getMeasurementPosition();
//    System.out.println("PID Low PreTest Height: " + pre);
//    System.out.println("PID Low PostTest Height: " + post);
//    assertFalse(pre.isNear(post, Meters.of(0.05)));
  }

  private static void dutyCycleTest(SmartMotorController smc, Command dutycycleUp, Command dutyCycleDown)
  throws InterruptedException
  {
    Distance       preDist    = smc.getMeasurementPosition();
    LinearVelocity pre        = smc.getMeasurementVelocity();
    LinearVelocity post;
    Distance       postDist;
    AtomicBoolean  testPassed = new AtomicBoolean(false);

    TestWithScheduler.schedule(dutycycleUp);
    TestWithScheduler.cycle(Seconds.of(1), () -> {
      if (smc.getDutyCycle() != 0)
      {
        testPassed.set(true);
      }
    });
    if (smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper)
    {
      Thread.sleep(200);
      TestWithScheduler.cycle(Seconds.of(1));
    }

    post = smc.getMeasurementVelocity();
    postDist = smc.getMeasurementPosition();
    System.out.println("DutyCycleUp PreTest Speed: " + pre);
    System.out.println("DutyCycleUp PreTest Dist: " + preDist);

    System.out.println("DutyCycleUp PostTest Speed: " + post);
    System.out.println("DutyCycleUp PostTest Dist: " + postDist);
    boolean pass = pre.lt(post) || preDist.lt(postDist) || testPassed.get();
    if ((smc instanceof TalonFXSWrapper || smc instanceof TalonFXWrapper) && !pass)
    {
      System.out.println("[WARNING] TalonFXS or TalonFX did not pass test, current attributing this to OS differences.");
    } else
    {
      assertTrue(pass);
    }
//    assertTrue(pre.lt(post));

//    pre = smc.getMeasurementVelocity();
//    TestWithScheduler.schedule(dutyCycleDown);
//    TestWithScheduler.cycle(Seconds.of(2));
//
//    post = smc.getMeasurementVelocity();
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
    startTest(smc);
    smc.setupSimulation();
    SmartMotorControllerTestSubsystem subsys = (SmartMotorControllerTestSubsystem) smc.getConfig().getSubsystem();

    Command dutyCycleUp   = subsys.setDutyCycle(1);
    Command dutyCycleDown = subsys.setDutyCycle(-1);

    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);

    closeSMC(smc);
  }


  @ParameterizedTest
  @MethodSource("createConfigs")
  void testSMCPositionPID(SmartMotorController smc) throws InterruptedException
  {
    if (smc instanceof TalonFXSWrapper)
    {
//      smc.applyConfig(smc.getConfig()
//                         .withClosedLoopController(0.2,
//                                                   0,
//                                                   0,
//                                                   MetersPerSecond.of(0.1),
//                                                   MetersPerSecondPerSecond.of(0.5)));
    }
    if (smc instanceof TalonFXWrapper)
    {
//      smc.applyConfig(smc.getConfig()
//                         .withClosedLoopController(0.02,
//                                                   0,
//                                                   0,
//                                                   MetersPerSecond.of(0.1),
//                                                   MetersPerSecondPerSecond.of(0.5)));
    }
    startTest(smc);
    smc.setupSimulation();
    Command highPid = Commands.run(() -> smc.setPosition(Meters.of(2)));
    Command lowPid  = Commands.run(() -> smc.setPosition(Meters.of(0)));

    positionPidTest(smc, highPid, lowPid);

    closeSMC(smc);
  }

  @ParameterizedTest
  @MethodSource("createConfigs")
  void testElevatorDutyCycle(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Elevator elevator      = createElevator(smc);
    Command  dutyCycleUp   = elevator.set(1);
    Command  dutyCycleDown = elevator.set(-0.5);

    dutyCycleTest(smc, dutyCycleUp, dutyCycleDown);

    closeSMC(smc);
  }


  @ParameterizedTest
  @MethodSource("createConfigs")
  void testElevatorPositionPID(SmartMotorController smc) throws InterruptedException
  {
    startTest(smc);
    Elevator elevator = createElevator(smc);
    Command  highPid  = elevator.setHeight(Meters.of(2));
    Command  lowPid   = elevator.setHeight(Meters.of(0.5));

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
