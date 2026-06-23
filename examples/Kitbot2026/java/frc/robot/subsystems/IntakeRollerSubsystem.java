// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Amps;

import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Duty-cycle roller for the Kitbot 2026 intake. Spins one Neo Vortex to
 * pull game pieces in or push them out.
 *
 * <p>This subsystem uses the raw SparkMax API instead of the YAMS
 * SmartMotorController wrapper. The intake roller only ever runs at a fixed
 * duty-cycle -- no position or velocity setpoint, no feedforward, no
 * simulation model that needs encoder feedback. Adding the YAMS wrapper
 * would bring closed-loop plumbing and telemetry overhead with no benefit
 * for a simple on/off actuator like this. The raw API is the right tool here.
 *
 * <p>Simulation is handled manually via FlywheelSim, which is enough to
 * exercise the current-sense path used by outtaking().
 */
public class IntakeRollerSubsystem extends SubsystemBase
{
  // 0.00032 kg*m^2 is a reasonable estimate for a small roller (light, short radius).
  // Used only in simulation -- does not affect real hardware behavior.
  public static final double kWristMomentOfInertia = 0.00032; // kg * m^2

  // CAN ID 30. Neo Vortex is brushless; kBrushless is required for SparkMax.
  private final SparkMax m_rollerMotor = new SparkMax(30, MotorType.kBrushless);

  private final DCMotor m_rollerMotorGearbox = DCMotor.getNeoVortex(1);

  // 1:1 gearing on the roller -- no gearbox, direct drive from the motor shaft.
  // The 1.0/4096.0 argument is the encoder counts-per-revolution conversion;
  // it is unused here but required by the FlywheelSim constructor.
  private final FlywheelSim m_rollerSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(
      m_rollerMotorGearbox,
      kWristMomentOfInertia,
      1), m_rollerMotorGearbox, 1.0 / 4096.0);

  private final SparkMaxSim m_rollerMotorSim = new SparkMaxSim(m_rollerMotor, m_rollerMotorGearbox);


  public IntakeRollerSubsystem()
  {
    SparkMaxConfig config = new SparkMaxConfig();
    config
        .inverted(false)
        // 100 A limit is high for a small roller but the Neo Vortex can sustain it
        // briefly. This protects the motor from a permanent stall without cutting
        // power at normal intake loads.
        .smartCurrentLimit(100);
    // COAST so the roller spins down freely after a command ends; no need to hold
    // position and BRAKE mode would jerk game pieces at the moment of release.
    config.idleMode(IdleMode.kCoast);
    // kNoResetSafeParameters preserves any settings already on the controller from
    // a prior boot; kPersistParameters writes these values to flash so they survive
    // a power cycle without re-running this constructor.
    m_rollerMotor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void simulationPeriodic()
  {
    // Feed the applied output (as a fraction of bus voltage) into the physics model.
    m_rollerSim.setInput(m_rollerMotorSim.getAppliedOutput() * RoboRioSim.getVInVoltage());

    // Advance the sim by one 20 ms control loop tick.
    m_rollerSim.update(0.02);

    // Push the sim's angular velocity back into the SparkMax sim so that
    // getOutputCurrent() and other sensor reads return plausible values.
    m_rollerMotorSim.iterate(m_rollerSim.getAngularVelocityRPM(),
                             RoboRioSim.getVInVoltage(),
                             0.02);
  }

  /**
   * Runs the roller at the given duty cycle. Positive values pull game pieces in.
   */
  public Command setIntakeRoller(double speed)
  {
    return runOnce(() -> {
      m_rollerMotor.set(speed);
    });
  }

  /**
   * Spin the roller in the outtake direction. Speed is negated so that positive
   * caller values always mean "push out."
   */
  public Command out(double speed)
  {
    return setIntakeRoller(speed * -1);
  }

  /** Spin the roller in the intake direction. */
  public Command in(double speed)
  {
    return setIntakeRoller(speed);
  }

  public Command stop(){
    return setIntakeRoller(0);
  }

  /** Returns the stator current reported by the SparkMax. Useful for game-piece detection. */
  public Current getCurrent()
  {
    return Amps.of(m_rollerMotor.getOutputCurrent());
  }

  /**
   * Returns true when the roller is actively ejecting. Checks both the applied
   * output (positive = outtake direction) and the command name so that named
   * auto commands like "Outtake" are also detected correctly.
   */
  public boolean outtaking()
  {
    if(getCurrentCommand() != null)
        return getDutycycle() > 0.0 || getCurrentCommand().getName().equals("Outtake");
    return getDutycycle() > 0.0;
  }

  public double getDutycycle()
  {
    return m_rollerMotor.getAppliedOutput();
  }
}
