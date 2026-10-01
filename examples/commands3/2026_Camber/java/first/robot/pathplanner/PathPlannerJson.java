// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later
// Ported from Team 9658's 2026-KitBot (https://github.com/9658-Camber-Robotics/2026-KitBot).

package first.robot.pathplanner;

import io.avaje.json.mapper.JsonMapper;
import java.util.Map;

/** Reads PathPlanner's JSON files with the avaje JSON library that ships with WPILib 2027. */
final class PathPlannerJson
{

  private static final JsonMapper MAPPER = JsonMapper.builder().build();

  private PathPlannerJson()
  {
  }

  /**
   * Parse a JSON object into maps, lists, strings, numbers, booleans, and nulls.
   *
   * @param json JSON text.
   * @return The parsed object.
   */
  static Map<String, Object> readObject(String json)
  {
    return MAPPER.fromJsonObject(json);
  }
}
