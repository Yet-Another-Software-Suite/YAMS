// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands2.config;

import java.util.Optional;
import org.wpilib.command2.Subsystem;
import yams.commands2.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.motorcontrollers.SmartMotorControllerConfig} that
 * adds the {@link Subsystem} binding and command-layer helpers (e.g. live tuning) that core must
 * not depend on.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
 *     .withMotorInverted(false)
 *     .withStatorCurrentLimit(Amps.of(40));
 * config.setupLiveTuning();
 * }</pre>
 *
 */
public class SmartMotorControllerConfig extends yams.core.motorcontrollers.SmartMotorControllerConfig {
  /** Subsystem that the {@link SmartMotorController} controls. */
  private Optional<Subsystem> subsystem = Optional.empty();

  /**
   * Construct the {@link SmartMotorControllerConfig} for the {@link Subsystem}
   *
   * @param subsystem {@link Subsystem} to use.
   */
  public SmartMotorControllerConfig(Subsystem subsystem) {
    super();
    this.subsystem = Optional.ofNullable(subsystem);
  }

  /**
   * Construct the {@link SmartMotorControllerConfig} with a {@link Subsystem} added later.
   *
   * @implNote You must use {@link #withSubsystem(Subsystem)} before passing off to {@link
   *           SmartMotorController}
   */
  public SmartMotorControllerConfig() {
    super();
  }

  /**
   * Duplicate the SmartMotorControllerConfig.
   *
   * @param cfg Config to duplicate.
   */
  private SmartMotorControllerConfig(SmartMotorControllerConfig cfg) {
    super(cfg);
    this.subsystem = cfg.subsystem;
  }

  @Override
  public SmartMotorControllerConfig clone() {
    return new SmartMotorControllerConfig(this);
  }

  /**
   * Sets the {@link Subsystem} for the {@link SmartMotorControllerConfig} to pass along to {@link
   * SmartMotorController} and the mechanisms built on it. Must be set if a {@link Subsystem} was
   * not defined previously.
   *
   * @param subsystem {@link Subsystem} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote Does not copy the entire config, should NEVER be reused.
   */
  public SmartMotorControllerConfig withSubsystem(Subsystem subsystem) {
    if (this.subsystem.isPresent()) {
      throw new SmartMotorControllerConfigurationException("Subsystem has already been set", "Cannot set subsystem", "withSubsystem(Subsystem subsystem) should only be called once");
    }
    this.subsystem = Optional.of(subsystem);
    return this;
  }

  /**
   * Get the subsystem controlled by the {@link SmartMotorController}
   *
   * @return {@link Subsystem} controlled.
   */
  public Subsystem getSubsystem() {
    if (subsystem.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Subsystem is undefined", "Subsystem cannot be created.", "withSubsystem(Subsystem)");
    }
    return subsystem.orElseThrow();
  }

  /**
   * Set up live tuning for the {@link SmartMotorController} this config is attached to: registers
   * a shared "Live Tuning" {@link org.wpilib.command2.Command} on {@link #getSubsystem()} via
   * {@link SmartMotorControllerCommandRegistry} that pulls tuned values from NetworkTables each
   * loop, and registers cleanup so the registration is removed when the controller is closed.
   *
   * <p>No-ops if the subsystem has not been set, or if this config has not yet been attached to a
   * {@link SmartMotorController} (i.e. {@code setupTelemetry()} has not been called on the
   * controller yet).
   */
  public void setupLiveTuning() {
    if (subsystem.isEmpty()) {
      return;
    }
    getAttachedController().ifPresent(controller -> {
      Subsystem sub = subsystem.get();
      SmartMotorControllerCommandRegistry.addCommand("Live Tuning", sub, controller::applyTuningValues);
      controller.addCloseHook(() -> SmartMotorControllerCommandRegistry.removeCommands(sub));
    });
  }
}
