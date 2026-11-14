// Copyright (c) 2026 Yet Another Software Suite
// SPDX-License-Identifier: LGPL-3.0-or-later

package yams.core;

import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import java.util.concurrent.ThreadLocalRandom;
import java.util.concurrent.atomic.AtomicBoolean;
import org.wpilib.driverstation.DriverStationDisplay;

/**
 * Rivet, the YASS mascot, says hi on the SystemCore display when YAMS starts up.
 *
 * <p>The art is written once, the first time a YAMS motor controller config or swerve drive is
 * built. It uses {@link DriverStationDisplay} line mode, so robot code that writes its own lines
 * afterwards simply replaces it.
 */
public final class Rivet {

  /**
   * Rivet, traced from the official pixel art with two characters per pixel. Each glyph stands for
   * one color: {@code #} pupils, {@code @} eye whites, {@code %} body, {@code 8} feet
   * shading, {@code +} belly, {@code O} and {@code o} monocle, {@code |} monocle chain, {@code ~}
   * and {@code =} mustache, {@code v} mouth.
   */
  private static final String[] ART = {
      "          %%%%%%%%                    %%%%%%%%",
      "        %%%%%%%%%%%%                %%OOOOOOOO%%%%",
      "        %%@@@@@@@@%%%%%%%%%%%%%%%%%%OOooooooooOO%%",
      "      %%@@@@####@@@@%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%",
      "      %%@@########@@%%%%%%%%%%%%OOoo@@@@####@@@@ooOO",
      "      %%@@########@@%%%%%%%%%%%%OOoo@@########@@ooOO",
      "    %%%%@@@@####@@@@%%%%%%%%%%%%OOoo@@########@@ooOO",
      "  %%%%%%%%@@@@@@@@%%%%%%%%%%%%%%OOoo@@@@####@@@@ooOO",
      "  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOoo@@@@@@@@ooOO%%%%",
      "  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%OOooooooooOO||%%%%",
      "%%%%%%%%%%%%%%%%~~~~~~%%~~~~~~~~%%%%%%OOOOOOOO%%%%||%%",
      "%%%%~~%%%%%%%%~~======~~======~~~~%%%%%%%%%%~~%%%%||%%",
      "%%%%vv~~%%%%~~========~~==========~~~~%%%%~~vv%%%%||%%",
      "  %%%%%%~~~~========~~~~~~vv==========~~vv%%%%%%||%%%%",
      "  %%%%%%vv========~~vv%%vvvv==========vv%%%%%%%%%%%%",
      "  %%%%%%%%vvvvvvvvvv%%%%%%vvvvvvvvvvvv%%%%%%%%%%%%",
      "      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
      "      %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
      "        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%",
      "        %%%%%%%%++++++++++++++++%%%%%%%%%%%%%%%%%%%%",
      "        %%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%",
      "        %%%%%%++++++++++++++++++++%%%%%%%%%%%%%%%%%%",
      "      %%%%%%%%++++++++++++++++++++%%%%%%%%%%88%%%%%%",
      "      %%%%88%%%%++++++++++++++++%%%%%%%%%%%%88%%%%%%",
      "    %%%%8888%%%%%%++++++++++++88%%%%%%%%%%%%88%%%%%%",
      "    888888  %%%%%%            88%%%%%%%%%%  88888888",
  };

  /**
   * What Rivet might say under the art; one is picked at random each startup. The first thirteen
   * are Rivet's speech bubbles from yassrobotics.com/rivet.
   */
  private static final String[] PHRASES = {
      "I lick batteries for fun.",
      "I programmed an arm today :)",
      "This mustache is 100% real.",
      "Ask me about YAGSL sometime.",
      "I once inspected 40 robots in one day.",
      "Simulation first, always.",
      "My pronouns change when I feel like it.",
      "That tickles.",
      "Free software, free frog.",
      "I'm basically a CSA now.",
      "Pi Day is my Super Bowl.",
      "Rivet was here.",
      "I may have eaten a battery once.",
      "Ribbit!",
      "Is this battery charged? Only one way to find out.",
      "Have you checked your CAN wiring?",
      "Clipboard in hand, tape measure ready.",
      "Whistle on, penalty stick ready.",
      "Hop in, we're going for a joyride.",
      "Got a sticker? That's my face!",
      "Spare bolts make great stocking stuffers.",
      "Gear ratios are my love language.",
      "Did you tune your PIDs today?",
      "Part leprechaun, all frog.",
      "Bumpers on? Let's play!",
  };

  /** Credit after the phrase. */
  private static final String SIGNATURE = " - Rivet";

  /** Row of {@link #ART} the YAMS version is printed beside, level with the mustache. */
  private static final int VERSION_ROW = 12;

  /** Spaces between the widest row of {@link #ART} and the text beside it. */
  private static final int VERSION_GAP = 4;

  /** Printed under the YAMS version. */
  private static final String WEBSITE = "yassrobotics.com";

  /** Resource, next to this class, that the build writes the YAMS version into. */
  private static final String VERSION_RESOURCE = "yams-version.txt";

  /** Version shown when YAMS was built from source rather than a release. */
  private static final String DEV_VERSION = "dev";

  /** Whether Rivet has already been shown. */
  private static final AtomicBoolean shown = new AtomicBoolean(false);

  private Rivet() {}

  /**
   * Show Rivet on the display, with the YAMS version and yassrobotics.com to the right of Rivet and
   * a random phrase underneath. Only the first call does anything.
   */
  public static void show() {
    if (!shown.compareAndSet(false, true)) {
      return;
    }
    try {
      int artWidth = Arrays.stream(ART).mapToInt(String::length).max().orElse(0);
      String[] side = {"\033[97mYAMS " + versionLabel(), "\033[92m" + WEBSITE};
      for (int row = 0; row < ART.length; row++) {
        String line = colorize(ART[row]);
        int sideRow = row - VERSION_ROW;
        if (sideRow >= 0 && sideRow < side.length) {
          line += " ".repeat(artWidth - ART[row].length() + VERSION_GAP) + side[sideRow];
        }
        DriverStationDisplay.addLine(line);
      }
      String caption =
          PHRASES[ThreadLocalRandom.current().nextInt(PHRASES.length)] + SIGNATURE;
      DriverStationDisplay.addLine(
          " ".repeat(Math.max(0, (artWidth - caption.length()) / 2)) + caption);
      DriverStationDisplay.updateLines();
    } catch (RuntimeException | LinkageError e) {
      // The splash is a nicety; never stop a robot because the display is unavailable.
    }
  }

  /**
   * Get the YAMS version this library was built as.
   *
   * @return The release version, such as {@code 2026.11.14}, or {@code dev} when YAMS was built
   *     from source.
   */
  public static String getVersion() {
    try (InputStream in = Rivet.class.getResourceAsStream(VERSION_RESOURCE)) {
      if (in != null) {
        String version = new String(in.readAllBytes(), StandardCharsets.UTF_8).trim();
        if (!version.isEmpty()) {
          return version;
        }
      }
    } catch (IOException e) {
      // Fall through to the source build version.
    }
    return DEV_VERSION;
  }

  /**
   * The version as shown beside Rivet, with a {@code v} in front of release numbers.
   *
   * @return Version label, such as {@code v2026.11.14} or {@code dev}.
   */
  private static String versionLabel() {
    String version = getVersion();
    return Character.isDigit(version.charAt(0)) ? "v" + version : version;
  }

  /**
   * Color a line of the art, switching the ANSI color only where the glyph changes.
   *
   * @param line Plain ASCII line.
   * @return The line with ANSI colors.
   */
  private static String colorize(String line) {
    StringBuilder colored = new StringBuilder();
    char previous = 0;
    for (char glyph : line.toCharArray()) {
      if (glyph != previous && glyph != ' ') {
        colored.append(color(glyph));
        previous = glyph;
      }
      colored.append(glyph);
    }
    return colored.toString();
  }

  /**
   * ANSI color for a glyph, close to the color it stands for in the pixel art.
   *
   * @param glyph Glyph from {@link #ART}.
   * @return ANSI color escape.
   */
  private static String color(char glyph) {
    return switch (glyph) {
      case '#' -> "\033[90m";
      case '@' -> "\033[97m";
      case '%', '+' -> "\033[92m";
      case '8' -> "\033[32m";
      case 'O' -> "\033[93m";
      case 'o', '|' -> "\033[33m";
      case '~', '=', 'v' -> "\033[31m";
      default -> "\033[0m";
    };
  }
}
