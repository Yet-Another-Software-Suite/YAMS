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
import org.wpilib.hardware.hal.HAL;

/**
 * Rivet, the YASS mascot, says hi on the Driver Station display when YAMS starts up.
 *
 * <p>The splash is written once, the first time a YAMS motor controller config or swerve drive is
 * built. The Display panel docked in the Driver Station shows about 26 columns by 9 rows, so it
 * gets a short welcome with the YAMS version, yassrobotics.com and the live tuner, plus an arrow to
 * the panel's pop-out button. Popping the display out (or scrolling right) shows Rivet at full
 * resolution with a random phrase from Rivet under it. It uses {@link DriverStationDisplay} line
 * mode, so robot code that writes its own lines afterwards simply replaces it.
 */
public final class Rivet {

  /** Ends just left of the docked panel's pop-out button, which sits over columns 21 to 25. */
  private static final String CLICK_FOR_MORE = "Click for more ---->";

  /** Shown under the YAMS version. */
  private static final String WEBSITE = "yassrobotics.com";

  /** Shown under the website, after a blank row. */
  private static final String[] TUNER = {"Don't forget to try", "tuning.yassrobotics.com"};

  /**
   * Rivet at full resolution, two characters per pixel, traced from the official Rivet pixel art
   * with the outline left off. Each glyph stands for one color: {@code #} pupils, {@code @} eye
   * whites, {@code %} body, {@code +} belly, {@code 8} feet shading, {@code O} and {@code o}
   * monocle, {@code |} monocle chain, {@code ~} and {@code =} handlebar mustache, {@code v} mouth.
   */
  private static final String[] FULL = {
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

  /** Column {@link #FULL} starts at, past the docked panel. */
  private static final int FULL_COLUMN = 30;

  /**
   * What Rivet might say under {@link #FULL}; one is picked at random each startup. Keep each to
   * three or four words. The first thirteen are Rivet's speech bubbles from yassrobotics.com/rivet,
   * trimmed.
   */
  private static final String[] PHRASES = {
      "I lick batteries.",
      "I programmed an arm.",
      "Mustache 100% real.",
      "Ask me about YAGSL.",
      "Inspected 40 robots today.",
      "Simulation first, always.",
      "My pronouns change often.",
      "Hey, that tickles!",
      "Free software, free frog.",
      "Basically a CSA now.",
      "Pi Day rules.",
      "Rivet was here.",
      "Ate a battery once.",
      "Ribbit ribbit ribbit!",
      "Battery charged? *lick*",
      "Need a CSA?",
      "Clipboard in hand.",
      "Whistle on, penalty ready.",
      "Hop in, joyride time!",
      "That sticker's my face!",
      "Bolts in my stocking.",
      "I love gear ratios.",
      "kP, kI, kD, kRibbit.",
      "Part leprechaun, all frog.",
      "Bumpers on? Let's play!",
  };

  /** Credit after the phrase. */
  private static final String SIGNATURE = " - Rivet";

  /** ANSI white, for text. */
  private static final String WHITE = "\033[97m";

  /** ANSI bright green, for the welcome. */
  private static final String GREEN = "\033[92m";

  /** Resource, next to this class, that the build writes the YAMS version into. */
  private static final String VERSION_RESOURCE = "yams-version.txt";

  /** Version shown when YAMS was built from source rather than a release. */
  private static final String DEV_VERSION = "dev";

  /** Whether Rivet has already been shown. */
  private static final AtomicBoolean shown = new AtomicBoolean(false);

  private Rivet() {}

  /**
   * Show the welcome and Rivet on the Driver Station display. Only the first call does anything.
   */
  public static void show() {
    if (!shown.compareAndSet(false, true)) {
      return;
    }
    try {
      // A config made before the robot starts, such as a static field in a unit test, comes before
      // the HAL is initialized, and writing to the display then crashes in native code. Initializing
      // it again does nothing.
      if (!HAL.initialize()) {
        return;
      }
      String[] welcome = {
          "Welcome to YASS!", CLICK_FOR_MORE, "", "YAMS " + versionLabel(), WEBSITE, "", TUNER[0],
          TUNER[1]
      };
      String phrase = PHRASES[ThreadLocalRandom.current().nextInt(PHRASES.length)] + SIGNATURE;
      int fullWidth = Arrays.stream(FULL).mapToInt(String::length).max().orElse(0);
      int rows = Math.max(welcome.length, FULL.length + 2);
      for (int row = 0; row < rows; row++) {
        String text = row < welcome.length ? welcome[row] : "";
        String left = text.isEmpty() ? "" : (row == 0 ? GREEN : WHITE) + text;
        String right = "";
        if (row < FULL.length) {
          right = colorize(FULL[row]);
        } else if (row == FULL.length + 1) {
          right = WHITE + " ".repeat(Math.max(0, (fullWidth - phrase.length()) / 2)) + phrase;
        }
        DriverStationDisplay.addLine(
            right.isEmpty() ? left : left + " ".repeat(FULL_COLUMN - text.length()) + right);
      }
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
   * The version as shown in the welcome, with a {@code v} in front of release numbers.
   *
   * @return Version label, such as {@code v2026.11.14} or {@code dev}.
   */
  private static String versionLabel() {
    String version = getVersion();
    return Character.isDigit(version.charAt(0)) ? "v" + version : version;
  }

  /**
   * Color a line of {@link #FULL}, switching the ANSI color only where the glyph changes.
   *
   * @param line Line of {@link #FULL}.
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
   * @param glyph Glyph from {@link #FULL}.
   * @return ANSI color escape.
   */
  private static String color(char glyph) {
    return switch (glyph) {
      case '#' -> "\033[90m";
      case '@' -> WHITE;
      case '%', '+' -> GREEN;
      case '8' -> "\033[32m";
      case 'O' -> "\033[93m";
      case 'o', '|' -> "\033[33m";
      case '~', '=', 'v' -> "\033[31m";
      default -> "\033[0m";
    };
  }
}
