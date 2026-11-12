// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core.motorcontrollers.config;

import io.avaje.jsonb.JsonType;
import io.avaje.jsonb.Jsonb;
import java.io.InputStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.wpilib.driverstation.internal.DriverStationBackend;

/**
 * Reads {@code deploy/adrc/<name>.json} into an {@link ADRCConfig}.
 *
 * <p><b>Written by FRC Team 8331</b>, who contributed ADRC to YAMS.
 *
 * <p>The file is parsed with Avaje Jsonb, the JSON library WPILib uses and ships with every robot
 * project, into plain maps and lists, so YAMS needs no generated adapter.
 *
 * <p>Failures are reported to the Driver Station and leave the Java defaults in place, so a missing
 * or malformed tune cannot stop the robot from starting. {@code systemOrder} is the exception: it
 * has no default, and {@link ADRCConfig#buildController} throws while it is unset. An unknown key, or
 * a value of the wrong type, is warned about and skipped without discarding the rest of the file.
 */
final class ADRCConfigLoader {
  /** Folder under the deploy directory that holds the per-controller JSON files. */
  static final String FOLDER = "adrc";

  /** The keys an ADRC file may set. */
  private static final Set<String> KEYS = Set.of("systemOrder", "settlingTime", "bandwidthScale", "b0", "minVoltage", "maxVoltage", "slewRateLimit", "inputDelay", "cascaded", "tdMethod",
                                                  "tdParams");

  /** Parses a file into maps, lists, strings, numbers and booleans. */
  private static final JsonType<Object> JSON = Jsonb.builder().build().type(Object.class);

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

    final Object parsed;
    try (InputStream stream = Files.newInputStream(file)) {
      parsed = JSON.fromJson(stream);
    } catch (Exception e) {
      warn("Could not read ADRC config " + file + ": " + e.getMessage() + ". Using Java defaults.");
      return;
    }
    if (!(parsed instanceof Map<?, ?> values)) {
      warn("ADRC config " + file + " is not a JSON object. Using Java defaults.");
      return;
    }

    for (Object key : values.keySet()) {
      if (!KEYS.contains(String.valueOf(key))) {
        warn("ADRC config " + file + " has an unknown key \"" + key + "\". Known keys will still be applied.");
      }
    }

    target.applyJson(new ADRCJsonConfig(integer(file, values, "systemOrder"), number(file, values, "settlingTime"), number(file, values, "bandwidthScale"), number(file, values, "b0"),
                                        number(file, values, "minVoltage"), number(file, values, "maxVoltage"), number(file, values, "slewRateLimit"), number(file, values, "inputDelay"),
                                        bool(file, values, "cascaded"), string(file, values, "tdMethod"), numbers(file, values, "tdParams")));
  }

  /**
   * Read a number.
   *
   * @param file   File, for warnings.
   * @param values Parsed file.
   * @param key    Key to read.
   * @return The number, or {@code null} when it is missing or not a number.
   */
  private static Double number(Path file, Map<?, ?> values, String key) {
    Object value = values.get(key);
    if (value == null) {
      return null;
    }
    if (value instanceof Number number) {
      return number.doubleValue();
    }
    wrongType(file, key, "a number", value);
    return null;
  }

  /**
   * Read a whole number.
   *
   * @param file   File, for warnings.
   * @param values Parsed file.
   * @param key    Key to read.
   * @return The number, or {@code null} when it is missing or not a whole number.
   */
  private static Integer integer(Path file, Map<?, ?> values, String key) {
    Double value = number(file, values, key);
    if (value == null) {
      return null;
    }
    if (value != Math.rint(value) || Math.abs(value) > Integer.MAX_VALUE) {
      wrongType(file, key, "a whole number", value);
      return null;
    }
    return value.intValue();
  }

  /**
   * Read a boolean.
   *
   * @param file   File, for warnings.
   * @param values Parsed file.
   * @param key    Key to read.
   * @return The boolean, or {@code null} when it is missing or not a boolean.
   */
  private static Boolean bool(Path file, Map<?, ?> values, String key) {
    Object value = values.get(key);
    if (value == null) {
      return null;
    }
    if (value instanceof Boolean bool) {
      return bool;
    }
    wrongType(file, key, "true or false", value);
    return null;
  }

  /**
   * Read a string.
   *
   * @param file   File, for warnings.
   * @param values Parsed file.
   * @param key    Key to read.
   * @return The string, or {@code null} when it is missing or not a string.
   */
  private static String string(Path file, Map<?, ?> values, String key) {
    Object value = values.get(key);
    if (value == null) {
      return null;
    }
    if (value instanceof String string) {
      return string;
    }
    wrongType(file, key, "a string", value);
    return null;
  }

  /**
   * Read an array of numbers.
   *
   * @param file   File, for warnings.
   * @param values Parsed file.
   * @param key    Key to read.
   * @return The numbers, or {@code null} when the key is missing or is not an array of numbers.
   */
  private static double[] numbers(Path file, Map<?, ?> values, String key) {
    Object value = values.get(key);
    if (value == null) {
      return null;
    }
    if (!(value instanceof List<?> list)) {
      wrongType(file, key, "an array of numbers", value);
      return null;
    }
    double[] result = new double[list.size()];
    for (int i = 0; i < result.length; i++) {
      if (!(list.get(i) instanceof Number number)) {
        wrongType(file, key, "an array of numbers", value);
        return null;
      }
      result[i] = number.doubleValue();
    }
    return result;
  }

  /**
   * Warn about a value of the wrong type, which is skipped.
   *
   * @param file     File.
   * @param key      Key.
   * @param expected What the value should be.
   * @param value    What it was.
   */
  private static void wrongType(Path file, String key, String expected, Object value) {
    warn("ADRC config " + file + ": \"" + key + "\" must be " + expected + ", got " + value + ". Ignoring.");
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
