package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.config.MechanismPositionConfig;
import yams.mechanisms.positional.Arm;
import yams.mechanisms.positional.DoubleJointedArm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.local.SparkWrapper;

import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Seconds;
import static yams.mechanisms.SmartMechanism.gearbox;
import static yams.mechanisms.SmartMechanism.gearing;

public class JointedArmSubsystem extends SubsystemBase {
    private final SparkMax lowerMotor         = new SparkMax(4, SparkLowLevel.MotorType.kBrushless);
    private final SparkMax upperMotor         = new SparkMax(5, SparkLowLevel.MotorType.kBrushless);
    private final SmartMotorControllerConfig lowerConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            //.withSoftLimit(Degrees.of(-30), Degrees.of(100))
            .withGearing(gearing(gearbox(3, 4)))
//      .withExternalEncoder(armMotor.getAbsoluteEncoder())
            .withIdleMode(SmartMotorControllerConfig.MotorMode.BRAKE)
            .withTelemetry("LowerMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new ArmFeedforward(0, 0, 0, 0))
            .withControlMode(SmartMotorControllerConfig.ControlMode.CLOSED_LOOP);
    private final SmartMotorControllerConfig upperConfig = lowerConfig
            .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
            //.withSoftLimit(Degrees.of(0), Degrees.of(100))
            .withTelemetry("UpperMotor", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMotorInverted(false)
            .withFeedforward(new ArmFeedforward(0, 0, 0, 0));
    private final SmartMotorController upperSMC            = new SparkWrapper(lowerMotor,
            DCMotor.getNEO(1),
            upperConfig);
    private final SmartMotorController lowerSMC = new SparkWrapper(upperMotor,
            DCMotor.getNEO(1),
            lowerConfig);

    private final ArmConfig lowerArmConfig = new ArmConfig(lowerSMC)
            .withLength(Meters.of(0.135))
            .withHardLimit(Degrees.of(-100), Degrees.of(200))
            .withTelemetry("LowerArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(1))
            .withStartingPosition(Degrees.of(0));
    private final ArmConfig upperArmConfig = new ArmConfig(upperSMC)
            .withLength(Meters.of(0.135))
            .withHardLimit(Degrees.of(-100), Degrees.of(200))
            .withTelemetry("UpperArm", SmartMotorControllerConfig.TelemetryVerbosity.HIGH)
            .withMass(Pounds.of(1))
            .withSimColor(new Color8Bit(Color.kDarkRed))
            .withStartingPosition(Degrees.of(0));
    private final DoubleJointedArm jointedArm = new DoubleJointedArm(lowerArmConfig,  upperArmConfig);

    public static StructPublisher<Pose2d> testPose = NetworkTableInstance.getDefault()
            .getTable("SmartDashboard")
            .getStructTopic("Test Pose", Pose2d.struct)
            .publish();

    public JointedArmSubsystem() {
        testPose.set(new Pose2d(new Translation2d(1, 4), Rotation2d.kZero));
    }

    public void periodic()
    {
        jointedArm.updateTelemetry();
    }

    public void simulationPeriodic()
    {
        jointedArm.simIterate();
    }

    public Command sysId()
    {
        return jointedArm.sysId(Volts.of(3), Volts.of(3).per(Second), Second.of(30));
    }
}
