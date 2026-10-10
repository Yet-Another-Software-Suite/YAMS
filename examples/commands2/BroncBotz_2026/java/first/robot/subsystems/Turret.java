// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Not on the original BroncBotz 3481 robot. Added to show EasyCRT on a turret.

package first.robot.subsystems;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.DegreesPerSecond;
import static org.wpilib.units.Units.DegreesPerSecondPerSecond;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Pounds;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import first.robot.Constants.TurretConstants;
import first.robot.Ports;
import java.util.Optional;
import org.wpilib.command2.Command;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.controller.SimpleMotorFeedforward;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.Angle;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;
import yams.commands2.config.SmartMotorControllerConfig;
import yams.commands2.mechanisms.Pivot;
import yams.core.gearing.MechanismGearing;
import yams.core.mechanisms.config.PivotConfig;
import yams.core.motorcontrollers.SmartMotorController;
import yams.core.motorcontrollers.enums.ControlMode;
import yams.core.motorcontrollers.enums.MotorMode;
import yams.core.motorcontrollers.local.SparkWrapper;
import yams.core.telemetry.enums.TelemetryVerbosity;
import yams.core.units.EasyCRT;

/**
 * Turret, modeled as a YAMS {@link Pivot}. It turns more than once, so no single absolute encoder
 * knows where it is at startup. Two CANcoders on pinions with different tooth counts both mesh with
 * the turret ring, and {@link EasyCRT} turns their two readings into the turret angle.
 */
public class Turret extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Ports.kCANPort, Ports.kTurret, MotorType.kBrushless);

    // Zero both CANcoders in Phoenix Tuner X with the turret facing forward, and set their direction
    // so they count up when the turret turns counterclockwise.
    private final CANcoder encoderA = new CANcoder(Ports.kTurretEncoderA, Ports.kCTRECANBus);
    private final CANcoder encoderB = new CANcoder(Ports.kTurretEncoderB, Ports.kCTRECANBus);

    // 200 tooth ring with 19 and 21 tooth pinions: unique over lcm(19, 21) / 200 = 1.995 turns.
    private final EasyCRT crt = new EasyCRT(TurretConstants.kRingTeeth, TurretConstants.kEncoderATeeth, TurretConstants.kEncoderBTeeth);

    private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
        .withControlMode(ControlMode.CLOSED_LOOP)
        .withClosedLoopController(20, 0, 0)
        .withTrapezoidalProfile(DegreesPerSecond.of(720), DegreesPerSecondPerSecond.of(1440))
        .withGearing(new MechanismGearing(TurretConstants.kGearing))
        .withZeroPower(MotorMode.BRAKE)
        .withStatorCurrentLimit(Amps.of(40))
        .withVoltageCompensation(Volts.of(12))
        .withMotorInverted(false)
        .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
        .withSoftLimits(TurretConstants.kMin, TurretConstants.kMax)
        .withStartingPosition(TurretConstants.kForward)
        .withMomentOfInertia(Inches.of(8), Pounds.of(12))
        .withTelemetry("TurretMotor", TelemetryVerbosity.HIGH);

    private final SmartMotorController motorController = new SparkWrapper(motor, DCMotor.getNEO(1), motorConfig);

    private final Pivot turret = new Pivot(new PivotConfig()
        .withHardLimits(TurretConstants.kHardMin, TurretConstants.kHardMax)
        .withTelemetry("Turret", TelemetryVerbosity.HIGH),
        motorController);

    private final Alert crtFailedAlert = new Alert("Turret", "TurretCRT",
        "Turret absolute encoders disagree. The turret is assumed to be facing forward.", Level.HIGH);

    public Turret() {
        // The simulated turret starts at its configured starting position instead.
        if (RobotBase.isReal()) {
            crtFailedAlert.set(!seedFromAbsoluteEncoders());
        }
    }

    /**
     * Read both absolute encoders and tell the motor where the turret is. Only call this while the
     * turret is still: the two readings are taken one after the other.
     *
     * @return Whether the two encoders agreed on an angle.
     */
    public boolean seedFromAbsoluteEncoders() {
        return seed(
            encoderA.getAbsolutePosition().waitForUpdate(0.25).getValue(),
            encoderB.getAbsolutePosition().waitForUpdate(0.25).getValue());
    }

    /**
     * Tell the motor where the turret is from a pair of absolute encoder readings.
     *
     * @param encoderAReading Reading of the encoder on the 19 tooth pinion.
     * @param encoderBReading Reading of the encoder on the 21 tooth pinion.
     * @return Whether the two readings agreed on an angle.
     */
    public boolean seed(Angle encoderAReading, Angle encoderBReading) {
        Optional<Angle> angle = crt.solve(encoderAReading, encoderBReading);
        angle.ifPresent(motorController::setEncoderPosition);
        return angle.isPresent();
    }

    public Angle getAngle() {
        return turret.getAngle();
    }

    public void setAngleSetpoint(Angle angle) {
        turret.setMechanismPositionSetpoint(angle);
    }

    /** Hold an angle until interrupted. The default command holds it facing forward. */
    public Command holdAt(Angle angle) {
        return turret.setAngle(angle);
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
