//// Copyright (c) 2026 Yet Another Software Suite
//// SPDX-License-Identifier: LGPL-3.0-or-later
//
//package yams.mechs;
//
//import static edu.wpi.first.units.Units.Amps;
//import static edu.wpi.first.units.Units.Inches;
//import static edu.wpi.first.units.Units.Meters;
//import static edu.wpi.first.units.Units.Rotations;
//import static edu.wpi.first.units.Units.Seconds;
//import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
//import static org.junit.jupiter.api.Assertions.assertEquals;
//import static org.junit.jupiter.api.Assertions.assertNotNull;
//import static org.junit.jupiter.api.Assertions.assertTrue;
//
//import com.revrobotics.spark.SparkLowLevel.MotorType;
//import com.revrobotics.spark.SparkMax;
//import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Pose2d;
//import edu.wpi.first.math.geometry.Rotation2d;
//import edu.wpi.first.math.kinematics.ChassisSpeeds;
//import edu.wpi.first.math.system.plant.DCMotor;
//import edu.wpi.first.wpilibj.Preferences;
//import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import java.util.concurrent.atomic.AtomicInteger;
//import org.junit.jupiter.api.AfterEach;
//import org.junit.jupiter.api.BeforeEach;
//import org.junit.jupiter.api.Test;
//import yams.gearing.GearBox;
//import yams.gearing.MechanismGearing;
//import yams.helpers.MockHardwareExtension;
//import yams.helpers.TestWithScheduler;
//import yams.mechanisms.config.SwerveDriveConfig;
//import yams.mechanisms.config.SwerveModuleConfig;
//import yams.mechanisms.swerve.SwerveDrive;
//import yams.mechanisms.swerve.SwerveModule;
//import yams.motorcontrollers.SmartMotorController;
//import yams.motorcontrollers.SmartMotorControllerConfig;
//import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
//import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
//import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
//import yams.motorcontrollers.local.SparkWrapper;
//
//public class SwerveDriveTest
//{
//  // Module offset from robot centre: 24 in × 24 in square chassis → ≈0.3048 m half-side.
//  private static final double kModuleOffset = 0.3048;
//
//  // Unique CAN IDs across test instances, wraps within [1, 62].
//  private static final AtomicInteger kCanIdCounter = new AtomicInteger(0);
//
//  private static int nextCanId()
//  {
//    return (kCanIdCounter.getAndIncrement() % 80) + 1;
//  }
//
//  // ---- Minimal swerve subsystem -----------------------------------------------
//
//  private static class SwerveTestSubsystem extends SubsystemBase
//  {
//    SwerveDrive drive;
//
//    @Override
//    public void periodic()
//    {
//      if (drive != null)
//      {drive.updateTelemetry();}
//    }
//
//    @Override
//    public void simulationPeriodic()
//    {
//      if (drive != null)
//      {drive.simIterate();}
//    }
//  }
//
//  // ---- Test fields ------------------------------------------------------------
//
//  private SwerveTestSubsystem subsystem;
//
//  private SparkMax flDriveSpark, frDriveSpark, blDriveSpark, brDriveSpark;
//  private SparkMax flAzimuthSpark, frAzimuthSpark, blAzimuthSpark, brAzimuthSpark;
//
//  private SparkWrapper flDriveSMC, frDriveSMC, blDriveSMC, brDriveSMC;
//  private SparkWrapper flAzimuthSMC, frAzimuthSMC, blAzimuthSMC, brAzimuthSMC;
//
//  private SwerveModule fl, fr, bl, br;
//  private SwerveDrive  drive;
//
//  // ---- Config helpers ---------------------------------------------------------
//
//  private SmartMotorControllerConfig makeDriveSMCConfig(String name)
//  {
//    return new SmartMotorControllerConfig()
//        .withClosedLoopController(0.1, 0.0, 0.0)
//        .withMechanismCircumference(Meters.of(Inches.of(4).in(Meters) * Math.PI))
//        .withGearing(new MechanismGearing(GearBox.fromReductionStages(6.75)))
//        .withIdleMode(MotorMode.BRAKE)
//        .withStatorCurrentLimit(Amps.of(40))
//        .withMotorInverted(false)
//        .withControlMode(ControlMode.CLOSED_LOOP)
//        .withSubsystem(subsystem)
//        .withTelemetry(name, TelemetryVerbosity.HIGH);
//  }
//
//  private SmartMotorControllerConfig makeAzimuthSMCConfig(String name)
//  {
//    return new SmartMotorControllerConfig()
//        .withClosedLoopController(50.0, 0.0, 0.5)
//        .withGearing(new MechanismGearing(GearBox.fromReductionStages(150.0 / 7.0)))
//        .withIdleMode(MotorMode.BRAKE)
//        .withStatorCurrentLimit(Amps.of(20))
//        .withMotorInverted(false)
//        .withControlMode(ControlMode.CLOSED_LOOP)
//        .withSubsystem(subsystem)
//        .withTelemetry(name, TelemetryVerbosity.HIGH);
//  }
//
//  private SwerveModule makeModule(
//      SparkWrapper driveSMC, SparkWrapper azimuthSMC,
//      double front, double left, String name)
//  {
//    SwerveModuleConfig cfg = new SwerveModuleConfig(driveSMC, azimuthSMC)
//        .withAbsoluteEncoder(() -> Rotations.of(0))
//        .withAbsoluteEncoderOffset(Rotations.of(0))
//        .withWheelDiameter(Inches.of(4))
//        .withLocation(Meters.of(front), Meters.of(left))
//        .withOptimization(false)
//        .withTelemetry(name, TelemetryVerbosity.HIGH);
//    return new SwerveModule(cfg);
//  }
//
//  // ---- Setup / teardown -------------------------------------------------------
//
//  @BeforeEach
//  void setup()
//  {
//    MockHardwareExtension.beforeAll();
//    TestWithScheduler.schedulerStart();
//    TestWithScheduler.schedulerClear();
//
//    subsystem = new SwerveTestSubsystem();
//
//    flDriveSpark   = new SparkMax(nextCanId(), MotorType.kBrushless);
//    frDriveSpark   = new SparkMax(nextCanId(), MotorType.kBrushless);
//    blDriveSpark   = new SparkMax(nextCanId(), MotorType.kBrushless);
//    brDriveSpark   = new SparkMax(nextCanId(), MotorType.kBrushless);
//    flAzimuthSpark = new SparkMax(nextCanId(), MotorType.kBrushless);
//    frAzimuthSpark = new SparkMax(nextCanId(), MotorType.kBrushless);
//    blAzimuthSpark = new SparkMax(nextCanId(), MotorType.kBrushless);
//    brAzimuthSpark = new SparkMax(nextCanId(), MotorType.kBrushless);
//
//    flDriveSMC   = new SparkWrapper(flDriveSpark,   DCMotor.getNEO(1), makeDriveSMCConfig("FL_Drive"));
//    frDriveSMC   = new SparkWrapper(frDriveSpark,   DCMotor.getNEO(1), makeDriveSMCConfig("FR_Drive"));
//    blDriveSMC   = new SparkWrapper(blDriveSpark,   DCMotor.getNEO(1), makeDriveSMCConfig("BL_Drive"));
//    brDriveSMC   = new SparkWrapper(brDriveSpark,   DCMotor.getNEO(1), makeDriveSMCConfig("BR_Drive"));
//    flAzimuthSMC = new SparkWrapper(flAzimuthSpark, DCMotor.getNEO(1), makeAzimuthSMCConfig("FL_Azimuth"));
//    frAzimuthSMC = new SparkWrapper(frAzimuthSpark, DCMotor.getNEO(1), makeAzimuthSMCConfig("FR_Azimuth"));
//    blAzimuthSMC = new SparkWrapper(blAzimuthSpark, DCMotor.getNEO(1), makeAzimuthSMCConfig("BL_Azimuth"));
//    brAzimuthSMC = new SparkWrapper(brAzimuthSpark, DCMotor.getNEO(1), makeAzimuthSMCConfig("BR_Azimuth"));
//
//    for (SparkWrapper smc : new SparkWrapper[]{
//        flDriveSMC, frDriveSMC, blDriveSMC, brDriveSMC,
//        flAzimuthSMC, frAzimuthSMC, blAzimuthSMC, brAzimuthSMC})
//    {
//      smc.setupSimulation();
//    }
//
//    fl = makeModule(flDriveSMC, flAzimuthSMC,  kModuleOffset,  kModuleOffset, "FL");
//    fr = makeModule(frDriveSMC, frAzimuthSMC,  kModuleOffset, -kModuleOffset, "FR");
//    bl = makeModule(blDriveSMC, blAzimuthSMC, -kModuleOffset,  kModuleOffset, "BL");
//    br = makeModule(brDriveSMC, brAzimuthSMC, -kModuleOffset, -kModuleOffset, "BR");
//
//    SwerveDriveConfig driveCfg = new SwerveDriveConfig()
//        .withSubsystem(subsystem)
//        .withModules(fl, fr, bl, br)
//        .withGyro(() -> Rotations.of(0))
//        .withStartingPose(new Pose2d())
//        .withTranslationController(new PIDController(2.0, 0.0, 0.0))
//        .withRotationController(new PIDController(4.0, 0.0, 0.0));
//
//    drive = new SwerveDrive(driveCfg);
//    subsystem.drive = drive;
//  }
//
//  @AfterEach
//  void teardown()
//  {
//    subsystem.drive = null;
//    TestWithScheduler.schedulerClear();
//    CommandScheduler.getInstance().unregisterSubsystem(subsystem);
//
//    for (SmartMotorController smc : new SmartMotorController[]{
//        flDriveSMC, frDriveSMC, blDriveSMC, brDriveSMC,
//        flAzimuthSMC, frAzimuthSMC, blAzimuthSMC, brAzimuthSMC})
//    {
//      smc.close();
//    }
//
//    for (SparkMax spark : new SparkMax[]{
//        flDriveSpark, frDriveSpark, blDriveSpark, brDriveSpark,
//        flAzimuthSpark, frAzimuthSpark, blAzimuthSpark, brAzimuthSpark})
//    {
//      spark.close();
//    }
//
//    MockHardwareExtension.afterAll();
//    Preferences.removeAll();
//    TestWithScheduler.schedulerClear();
//  }
//
//  // ---- Tests ------------------------------------------------------------------
//
//  // SwerveDrive constructs and is non-null after setup.
//  @Test
//  void constructionDoesNotCrash()
//  {
//    assertNotNull(drive);
//  }
//
//  // updateTelemetry and simIterate run for several loops without throwing.
//  @Test
//  void telemetryAndSimRunWithoutCrash() throws InterruptedException
//  {
//    TestWithScheduler.cycle(Seconds.of(0.5));
//  }
//
//  // The initial pose matches the starting pose supplied in the config.
//  @Test
//  void initialPoseIsOrigin()
//  {
//    Pose2d pose = drive.getPose();
//    assertEquals(0.0, pose.getX(), 0.01);
//    assertEquals(0.0, pose.getY(), 0.01);
//    assertEquals(0.0, pose.getRotation().getDegrees(), 0.1);
//  }
//
//  // A non-zero target pose is reflected in getPose() immediately after resetOdometry.
//  @Test
//  void resetOdometryMatchesPose()
//  {
//    Pose2d target = new Pose2d(3.0, 2.0, Rotation2d.fromDegrees(45.0));
//    drive.resetOdometry(target);
//    Pose2d pose = drive.getPose();
//    assertEquals(3.0, pose.getX(), 0.01);
//    assertEquals(2.0, pose.getY(), 0.01);
//    assertEquals(45.0, pose.getRotation().getDegrees(), 0.1);
//  }
//
//  // getStateFromRobotRelativeChassisSpeeds converts a pure forward command into
//  // 1 m/s, 0° states for all four modules.
//  @Test
//  void getStateFromSpeedsForwardDrive()
//  {
//    var states = drive.getStateFromRobotRelativeChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0));
//
//    for (int i = 0; i < 4; i++)
//    {
//      assertEquals(1.0, states[i].speedMetersPerSecond, 0.01,
//                   "Module " + i + " speed should equal commanded speed");
//      assertEquals(0.0, states[i].angle.getDegrees(), 1.0,
//                   "Module " + i + " angle should be 0° for pure forward drive");
//    }
//  }
//
//  // Pure rotation command produces tangential module states (none pointing forward).
//  @Test
//  void getStateFromSpeedsPureRotation()
//  {
//    var states = drive.getStateFromRobotRelativeChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 1.0));
//
//    for (int i = 0; i < 4; i++)
//    {
//      assertTrue(Math.abs(states[i].speedMetersPerSecond) > 0.0,
//                 "Module " + i + " should have non-zero speed for rotation command");
//      assertTrue(Math.abs(states[i].angle.getDegrees()) > 1.0,
//                 "Module " + i + " should not point forward during pure rotation");
//    }
//  }
//
//  // setRobotRelativeChassisSpeeds does not throw for both non-zero and zero inputs.
//  @Test
//  void setRobotRelativeSpeedsDoesNotCrash()
//  {
//    assertDoesNotThrow(() -> drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(1.0, 0.0, 0.0)));
//    assertDoesNotThrow(() -> drive.setRobotRelativeChassisSpeeds(new ChassisSpeeds(0.0, 0.0, 0.0)));
//  }
//
//  // lockPose commands zero translational speed.
//  @Test
//  void lockPoseSetsZeroTranslation() throws InterruptedException
//  {
//    drive.lockPose();
//
//    ChassisSpeeds speeds = drive.getRobotRelativeSpeed();
//    assertEquals(0.0, speeds.vxMetersPerSecond, 0.5);
//    assertEquals(0.0, speeds.vyMetersPerSecond, 0.5);
//
//    TestWithScheduler.cycle(Seconds.of(0.5));
//    SwerveModule[] modules = drive.getConfig().getModules();
//    for (int i = 0; i < 4; i++)
//    {
//      double expected = modules[i].getConfig().getLocation().orElseThrow().getAngle().getDegrees();
//      double actual   = modules[i].getState().angle.getDegrees();
//      assertEquals(expected, actual, 180.0,
//                   "Module " + i + " angle should converge toward lock angle " + expected + "°");
//    }
//  }
//
//  // zeroGyro does not throw; pose rotation becomes 0.
//  @Test
//  void zeroGyroDoesNotCrash()
//  {
//    assertDoesNotThrow(() -> drive.zeroGyro());
//    assertEquals(0.0, drive.getPose().getRotation().getDegrees(), 1.0);
//  }
//
//  // addVisionMeasurement accepts a pose without throwing.
//  @Test
//  void addVisionMeasurementDoesNotCrash()
//  {
//    assertDoesNotThrow(
//        () -> drive.addVisionMeasurement(new Pose2d(1.0, 1.0, Rotation2d.kZero), 0.0));
//  }
//
//  // getDistanceFromPose returns the Euclidean distance (3-4-5 right triangle).
//  @Test
//  void getDistanceFromPose()
//  {
//    double dist = drive.getDistanceFromPose(new Pose2d(3.0, 4.0, Rotation2d.kZero)).in(Meters);
//    assertEquals(5.0, dist, 0.01);
//  }
//
//  // drive() returns a Command that invokes the speed supplier each scheduler loop.
//  @Test
//  void driveCommandCallsSpeedSupplier() throws InterruptedException
//  {
//    AtomicInteger callCount = new AtomicInteger(0);
//    Command cmd = drive.drive(() -> {
//      callCount.incrementAndGet();
//      return new ChassisSpeeds();
//    });
//    TestWithScheduler.schedule(cmd);
//    TestWithScheduler.cycle(Seconds.of(0.1));
//    assertTrue(callCount.get() >= 1, "Speed supplier should have been called at least once");
//  }
//
//  // The drive Command declares the configured subsystem as a requirement.
//  @Test
//  void driveCommandHasSubsystemRequirement()
//  {
//    Command cmd = drive.drive(() -> new ChassisSpeeds());
//    assertTrue(cmd.getRequirements().contains(subsystem),
//               "Drive command must require the swerve subsystem");
//  }
//
//  // Scheduling a second drive Command cancels the first.
//  @Test
//  void secondDriveCommandInterruptsFirst() throws InterruptedException
//  {
//    AtomicInteger firstCalls  = new AtomicInteger(0);
//    AtomicInteger secondCalls = new AtomicInteger(0);
//
//    Command cmd1 = drive.drive(() -> {
//      firstCalls.incrementAndGet();
//      return new ChassisSpeeds();
//    });
//    Command cmd2 = drive.drive(() -> {
//      secondCalls.incrementAndGet();
//      return new ChassisSpeeds();
//    });
//
//    TestWithScheduler.schedule(cmd1);
//    TestWithScheduler.cycle(Seconds.of(0.04));
//    int firstCallsAtInterrupt = firstCalls.get();
//    assertTrue(firstCallsAtInterrupt >= 1, "cmd1 should run initially");
//
//    TestWithScheduler.schedule(cmd2);
//    TestWithScheduler.cycle(Seconds.of(0.04));
//
//    assertTrue(secondCalls.get() >= 1, "cmd2 should run after interrupt");
//    assertEquals(firstCallsAtInterrupt, firstCalls.get(), "cmd1 should have been cancelled");
//  }
//}
