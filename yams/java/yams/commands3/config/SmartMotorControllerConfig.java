// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.commands3.config;

import java.util.Optional;
import org.wpilib.command3.Mechanism;
import yams.commands3.telemetry.SmartMotorControllerCommandRegistry;
import yams.core.exceptions.SmartMotorControllerConfigurationException;
import yams.core.motorcontrollers.SmartMotorController;

/**
 * Command-based extension of {@link yams.core.motorcontrollers.SmartMotorControllerConfig} that
 * adds the {@link Mechanism} binding and command-layer helpers (e.g. live tuning) that core must
 * not depend on.
 *
 * <h2>Example</h2>
 *
 * <pre>{@code
 * SmartMotorControllerConfig config = new SmartMotorControllerConfig(this)
 *     .withMotorInverted(false)
 *     .withStatorCurrentLimit(Amps.of(40));
 * }</pre>
 *
 */
public class SmartMotorControllerConfig extends yams.core.motorcontrollers.SmartMotorControllerConfig {
  /** Mechanism that the {@link SmartMotorController} controls. */
  private Optional<Mechanism> mechanism = Optional.empty();

  /**
   * Construct the {@link SmartMotorControllerConfig} for the {@link Mechanism}
   *
   * @param mechanism {@link Mechanism} to use.
   */
  public SmartMotorControllerConfig(Mechanism mechanism) {
    super();
    this.mechanism = Optional.ofNullable(mechanism);
  }

  /**
   * Construct the {@link SmartMotorControllerConfig} with a {@link Mechanism} added later.
   *
   * @implNote You must use {@link #withMechanism(Mechanism)} before passing off to {@link
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
    this.mechanism = cfg.mechanism;
  }

  @Override
  public SmartMotorControllerConfig clone() {
    return new SmartMotorControllerConfig(this);
  }

  /**
   * Sets the {@link Mechanism} for the {@link SmartMotorControllerConfig} to pass along to {@link
   * SmartMotorController} and the mechanisms built on it. Must be set if a {@link Mechanism} was
   * not defined previously.
   *
   * @param mechanism {@link Mechanism} to use.
   * @return {@link SmartMotorControllerConfig} for chaining.
   * @implNote Does not copy the entire config, should NEVER be reused.
   */
  public SmartMotorControllerConfig withMechanism(Mechanism mechanism) {
    if (this.mechanism.isPresent()) {
      throw new SmartMotorControllerConfigurationException("Mechanism has already been set", "Cannot set mechanism", "withMechanism(Mechanism mechanism) should only be called once");
    }
    this.mechanism = Optional.of(mechanism);
    return this;
  }

  /**
   * Get the mechanism controlled by the {@link SmartMotorController}
   *
   * @return {@link Mechanism} controlled.
   */
  public Mechanism getMechanism() {
    if (mechanism.isEmpty()) {
      throw new SmartMotorControllerConfigurationException("Mechanism is undefined", "Mechanism cannot be created.", "withMechanism(Mechanism)");
    }
    return mechanism.orElseThrow();
  }

  /**
   * Set up live tuning for the {@link SmartMotorController} this config is attached to: registers
   * a shared "Live Tuning" {@link org.wpilib.command3.Command} on {@link #getMechanism()} via
   * {@link SmartMotorControllerCommandRegistry} that pulls tuned values from NetworkTables each
   * loop, and registers cleanup so the registration is removed when the controller is closed.
   */
  @Override
  public void setupLiveTuning() {
    if (mechanism.isEmpty()) {
      return;
    }
    getAttachedController().ifPresent(controller -> {
      Mechanism m = mechanism.get();
      SmartMotorControllerCommandRegistry.addCommand("Live Tuning", m, controller::applyTuningValues);
      controller.addCloseHook(() -> SmartMotorControllerCommandRegistry.removeCommands(m));
    });
  }
}
