package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.ArmConfig;
import yams.mechanisms.positional.Arm;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class HoodSubsystem extends SubsystemBase {
    private final SparkMax hoodMotor = new SparkMax(2, MotorType.kBrushless);

    private final SmartMotorControllerConfig hoodMotorConfig = new SmartMotorControllerConfig(this)
            .withClosedLoopController(0.00016541, 0, 0, RPM.of(5000), RotationsPerSecondPerSecond.of(2500))
            .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
            .withIdleMode(MotorMode.COAST)
            .withTelemetry("HoodMotor", TelemetryVerbosity.HIGH)
            .withStatorCurrentLimit(Amps.of(40))
            .withMotorInverted(false)
            .withClosedLoopRampRate(Seconds.of(0.25))
            .withOpenLoopRampRate(Seconds.of(0.25))
            .withFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withSimFeedforward(new SimpleMotorFeedforward(0.27937, 0.089836, 0.014557))
            .withControlMode(ControlMode.CLOSED_LOOP);

    private final SmartMotorController hoodSMC = new SparkWrapper(hoodMotor, DCMotor.getNeo550(1), hoodMotorConfig);

    private final ArmConfig hoodConfig = new ArmConfig(hoodSMC)
            .withTelemetry("HoodMech", TelemetryVerbosity.HIGH)
            .withSoftLimits(Degrees.of(5), Degrees.of(100))
            .withHardLimit(Degrees.of(0), Degrees.of(120)); // The Hood can be modeled as an arm since it has a
                                                            // gravitational force acted upon based on the angle its in

    private final Arm hood = new Arm(hoodConfig);

    public HoodSubsystem() {

    }

    public Command setAngle(Angle angle) {
        return hood.setAngle(angle);
    }

    public Command setAngle(Supplier<Angle> angleSupplier) {
        return hood.setAngle(angleSupplier);
    }

    public Angle getAngle() {
        return hood.getAngle();
    }

    public Command sysId() {
        return hood.sysId(
                Volts.of(4.0), // maximumVoltage
                Volts.per(Second).of(0.5), // step
                Seconds.of(8.0) // duration
        );
    }

    public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
        return hood.set(dutyCycleSupplier);
    }

    public Command setDutyCycle(double dutyCycle) {
        return hood.set(dutyCycle);
    }

    @Override
    public void periodic() {
        hood.updateTelemetry();
    }

    @Override
    public void simulationPeriodic() {
        hood.simIterate();
    }
}
