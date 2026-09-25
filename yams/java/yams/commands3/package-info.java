// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

/**
 * WPILib Commands v3 layer for the YAMS (Yet Another Mechanism System) library.
 *
 * <p>This package binds the YAMS core mechanisms ({@link yams.core.mechanisms}) to
 * {@link org.wpilib.command3.Mechanism}s and adds {@link org.wpilib.command3.Command} and
 * {@link org.wpilib.command3.Trigger} factories. Only use it in robot programs that use the
 * Commands v3 vendordep; {@code yams.commands2} is the equivalent layer for Commands v2.
 *
 * <h2>Mechanism Lifecycle</h2>
 * <ol>
 * <li><b>Create a motor controller configuration</b> with
 * {@link yams.commands3.config.SmartMotorControllerConfig}, passing the
 * {@link org.wpilib.command3.Mechanism} that owns the motor. Commands created by the YAMS mechanism
 * require that {@link org.wpilib.command3.Mechanism}.</li>
 * <li><b>Create a mechanism configuration</b> from {@link yams.core.mechanisms.config} (e.g.
 * {@link yams.core.mechanisms.config.ArmConfig}, {@link yams.core.mechanisms.config.ElevatorConfig})
 * using the fluent {@code with*()} API.</li>
 * <li><b>Construct the mechanism</b> from {@link yams.commands3.mechanisms} (e.g.
 * {@link yams.commands3.mechanisms.Arm}, {@link yams.commands3.mechanisms.FlyWheel}) with the
 * mechanism configuration and the {@link yams.core.motorcontrollers.SmartMotorController}.</li>
 * <li><b>Command the mechanism</b> with setpoint factories such as {@code setAngle()},
 * {@code setHeight()} or {@code run()}. Each returns a named {@link org.wpilib.command3.Command}
 * that can be bound in an opmode, awaited from a coroutine, or scheduled directly.</li>
 * <li><b>React to conditions</b> with trigger factories ({@code near()}, {@code max()},
 * {@code min()}, {@code between()}, {@code lte()}, {@code gte()}) that return
 * {@link org.wpilib.command3.Trigger}s.</li>
 * <li><b>Update telemetry and simulation</b>: Commands v3 {@link org.wpilib.command3.Mechanism}s
 * have no periodic hook, so call {@code updateTelemetry()} and {@code simIterate()} on each YAMS
 * mechanism from the robot's periodic methods.</li>
 * </ol>
 *
 * <h2>Typical Usage</h2>
 * <pre>{@code
 * public class ArmMechanism implements Mechanism {
 *   // 1. Configure the motor controller for this Mechanism
 *   private final SmartMotorControllerConfig motorConfig = new SmartMotorControllerConfig(this)
 *           .withClosedLoopController(4, 0, 0)
 *           .withGearing(new MechanismGearing(GearBox.fromReductionStages(3, 4)));
 *   private final SmartMotorController motor =
 *       new TalonFXWrapper(new TalonFX(1), DCMotor.getKrakenX60(1), motorConfig);
 *
 *   // 2. and 3. Configure and construct the YAMS mechanism
 *   private final Arm arm = new Arm(new ArmConfig()
 *       .withLength(Meters.of(0.135))
 *       .withHardLimits(Degrees.of(-100), Degrees.of(200)), motor);
 *
 *   // 4. Setpoint command, requires this Mechanism
 *   public Command stow() {
 *     return arm.setAngle(Degrees.of(0));
 *   }
 *
 *   // 5. Trigger
 *   public Trigger atStow() {
 *     return arm.near(Degrees.of(0), Degrees.of(2));
 *   }
 * }
 * }</pre>
 *
 * <h2>Sub-packages</h2>
 * <ul>
 * <li>{@link yams.commands3.config} configuration classes that bind a
 * {@link org.wpilib.command3.Mechanism}</li>
 * <li>{@link yams.commands3.mechanisms} command factories for Arm, Elevator, Pivot, FlyWheel,
 * DifferentialMechanism and DoubleJointedArm</li>
 * <li>{@link yams.commands3.swerve} command factories for the swerve drive</li>
 * <li>{@link yams.commands3.telemetry} live tuning commands published to the dashboard</li>
 * </ul>
 *
 * @see yams.core.mechanisms
 * @see yams.core.mechanisms.config
 * @see yams.core.motorcontrollers.SmartMotorController
 */
package yams.commands3;
