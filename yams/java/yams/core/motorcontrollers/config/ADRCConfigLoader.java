// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.config;

import io.avaje.jsonb.Jsonb;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import org.wpilib.driverstation.internal.DriverStationBackend;

/**
 * Reads {@code deploy/adrc/<name>.json} into an {@link ADRCConfig}.
 *
 * <p>Failures are reported to the Driver Station and leave the Java defaults in place, so a missing
 * or malformed tune cannot stop the robot from starting. {@code systemOrder} is the exception: it
 * has no default, and {@link ADRCConfig#buildController} throws while it is unset. Unknown keys are
 * warned about but do not discard the rest of the file.
 */
final class ADRCConfigLoader {
  /** Folder under the deploy directory that holds the per-controller JSON files. */
  static final String FOLDER = "adrc";

  /** Static helper, not instantiable. */
  private ADRCConfigLoader() {}

  /**
   * Apply the JSON file for {@code target} if it exists.
   *
   * @param target          Config to update.
   * @param deployDirectory Deploy directory that contains the {@value #FOLDER} folder.
   */
  static void load(ADRCConfig target, Path deployDirectory) {
    Path file = deployDirectory.resolve(FOLDER).resolve(target.getName() + ".json");
    if (!Files.isRegularFile(file)) {
      warn("ADRC config \"" + target.getName() + "\" was not found at " + file + ". Using Java defaults.");
      return;
    }

    warnOnUnknownKeys(target, file);

    try (InputStream stream = Files.newInputStream(file)) {
      ADRCJsonConfig json = jsonb(false).type(ADRCJsonConfig.class).fromJson(stream);
      if (json == null) {
        warn("ADRC config " + file + " was empty. Using Java defaults.");
        return;
      }
      target.applyJson(json);
    } catch (Exception e) {
      warn("Could not read ADRC config " + file + ": " + e.getMessage() + ". Using Java defaults.");
    }
  }

  /**
   * Re-read the file with unknown keys treated as errors, so a typo is visible in the logs.
   *
   * @param target Config being loaded.
   * @param file   File to check.
   */
  private static void warnOnUnknownKeys(ADRCConfig target, Path file) {
    try (InputStream stream = Files.newInputStream(file)) {
      jsonb(true).type(ADRCJsonConfig.class).fromJson(stream);
    } catch (Exception e) {
      warn("ADRC config " + file + " has an unknown or invalid key: " + e.getMessage() + ". Known keys will still be applied.");
    }
  }

  /**
   * Build a {@link Jsonb} that discovers the generated adapter through Avaje's service entry.
   *
   * <p>The class loader is set explicitly because the simulator loads robot code in a child class
   * loader that the thread context class loader does not see. Without it, the default {@link Jsonb}
   * is built without the generated adapter and every file fails with
   * {@code No JsonAdapter for class ADRCJsonConfig}.
   *
   * <p>{@code serializeEmpty(false)} is a write-only setting and is not used here; it is set only
   * to force Avaje to build a fresh instance instead of returning its cached default, which may
   * have been created before the generated adapter was visible.
   *
   * @param failOnUnknown Whether unknown keys should throw instead of being ignored.
   * @return Configured {@link Jsonb}.
   */
  private static Jsonb jsonb(boolean failOnUnknown) {
    return Jsonb.builder()
        .classLoader(ADRCJsonConfig.class.getClassLoader())
        .serializeEmpty(false)
        .failOnUnknown(failOnUnknown)
        .build();
  }

  /**
   * Report a config warning to the Driver Station and stderr.
   *
   * @param message Warning text.
   */
  static void warn(String message) {
    DriverStationBackend.reportWarning("[ADRC] " + message, false);
  }
}
