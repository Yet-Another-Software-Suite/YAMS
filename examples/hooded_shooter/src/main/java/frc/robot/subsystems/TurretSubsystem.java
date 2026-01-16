package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFXS;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.motorcontrollers.SmartMotorControllerConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import yams.gearing.GearBox;
import yams.gearing.MechanismGearing;
import yams.mechanisms.config.PivotConfig;
import yams.mechanisms.positional.Pivot;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.remote.TalonFXSWrapper;

public class TurretSubsystem extends SubsystemBase {
        private final TalonFXS turretMotor = new TalonFXS(1);
        private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
                        .withControlMode(ControlMode.CLOSED_LOOP)
                        .withClosedLoopController(4, 0, 0, DegreesPerSecond.of(180), DegreesPerSecondPerSecond.of(90))
                        // Configure Motor and Mechanism properties
                        .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)))
                        .withIdleMode(MotorMode.BRAKE)
                        .withMotorInverted(false)
                        // Setup Telemetry
                        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH)
                        // Power Optimization
                        .withStatorCurrentLimit(Amps.of(40))
                        .withClosedLoopRampRate(Seconds.of(0.25))
                        .withOpenLoopRampRate(Seconds.of(0.25));
        private final SmartMotorController turretSMC = new TalonFXSWrapper(turretMotor,
                        DCMotor.getNEO(1),
                        motorConfig);

        private final PivotConfig turretConfig = new PivotConfig(turretSMC)
                        .withStartingPosition(Degrees.of(0)) // Starting position of the Pivot
                        .withWrapping(Degrees.of(0), Degrees.of(360)) // Wrapping enabled bc the pivot can spin
                                                                      // infinitely
                        .withHardLimit(Degrees.of(0), Degrees.of(720)) // Hard limit bc wiring prevents infinite
                                                                       // spinning
                        .withTelemetry("TurretMech", TelemetryVerbosity.HIGH) // Telemetry
                        .withMOI(Meters.of(0.25), Pounds.of(4)); // MOI Calculation

        private final Pivot turret = new Pivot(turretConfig);

        public TurretSubsystem() {
        }

        public Command setAngle(Angle angle) {
                return turret.setAngle(angle);
        }

        public Command setAngle(Supplier<Angle> angleSupplier) {
                return turret.setAngle(angleSupplier);
        }

        public Angle getAngle() {
                return turret.getAngle();
        }

        public Command sysId() {
                return turret.sysId(
                                Volts.of(4.0), // maximumVoltage
                                Volts.per(Second).of(0.5), // step
                                Seconds.of(8.0) // duration
                );
        }

        public Command setDutyCycle(Supplier<Double> dutyCycleSupplier) {
                return turret.set(dutyCycleSupplier);
        }

        public Command setDutyCycle(double dutyCycle) {
                return turret.set(dutyCycle);
        }

        @Override
        public void periodic() {
                turret.updateTelemetry();
        }

        @Override
        public void simulationPeriodic() {
                turret.simIterate();
        }
}
